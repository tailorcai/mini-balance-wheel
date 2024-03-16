#pragma once

#include <WiFi.h>
#include <vector>
#include <string>
#include <iterator>
#include <mutex>

class Flex_Log {
    WiFiClient client;
    String  sIpHost;
    std::vector<String> vBuffers;
    std::mutex serial_mtx;
  public:
    Flex_Log() {}
    static Flex_Log& instance() {
        static Flex_Log singleton;
        return singleton;
    }

    // log to controller machine or just discard
    void log(const char* data) {
        // Serial.println( data );
        std::lock_guard<std::mutex> lck(serial_mtx);
        if( vBuffers.size() > 5)
            vBuffers.erase( vBuffers.begin() );
        vBuffers.push_back( String(data)+"\n" );
    }

    // log to controller machine or just discard
    void log(const String& str) {
        log(str.c_str());
    }

    void debug(const char* data) {
        Serial.println( data );
        log( data );
    }
    
    void debug(const String& str) {
        Serial.println( str );
        log(str.c_str());
    }

    bool begin(const char* ip_host) {
        sIpHost = String(ip_host);
        // 连接上位机
        if (!client.connect(sIpHost.c_str(), 1347)) {
            // Serial.println(String("Connecting to ") + sIpHost + " failed.");
            // Serial.println("Waiting 5 seconds before retrying...");
            // delay(5000);
            // return;
            return false;
        }
        return true;
    }

    void run( TaskFunction_t cmdFunc ) {
        static String in_buffer;
        while(1) {
            {
                String data;
                {
                    std::lock_guard<std::mutex> lck(serial_mtx);
                    for( const auto buf :vBuffers ) {
                        data += buf;
                    }
                    vBuffers.clear();
                }
                
                if( !data.isEmpty() ) {
                    client.write_P( data.c_str(), data.length() );
                }
            }
            if( client.available()) {
                // read from network
                uint8_t buf[100];
                auto len = client.read( buf, sizeof(buf));
                if( len > 0 ) { // !! -1 means fail
                    String got( (const char*) buf, len);
                    // Serial.println( String( "got: ") + len + "bytes to read, " + got );
                    in_buffer.concat( got );
                    while(in_buffer.length()>0) {
                        auto term = in_buffer.indexOf('\n');
                        if( term >= 0) {
                            auto line = in_buffer.substring(0,term+1);
                            in_buffer.remove(0,line.length());
                            // trigger signal to other task
                            // Serial.println( "got: " + line );
                            cmdFunc( (void*) line.c_str() );
                        }
                        else {
                            break;
                            // error happen, see 
                            // Serial.println( "ERR: buffer dump" );
                            // for( const auto c : in_buffer ) 
                            // Serial.printf("%x ", c );
                            // Serial.println("");
                        }
                    }
                }
            }
            delay(1);
        }
    }
};