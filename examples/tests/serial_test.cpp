//
// Created by masoud on 6/11/24.
//

#include <iostream>
#include <memory>
#include <string>
#include <chrono>
#include <serial/serial.h>
#include <iomanip>


using namespace std;

void test_speed(const shared_ptr<serial::Serial>& pSerial) {

    if (!pSerial) {
        cerr << "Null serial comm. detected, abort.\n";
        return;
    }

    int max_iter = 10000;
    ostringstream oss;
    string input_buff;
    int print_cnt = 100;
    double avg_rt = 0.0;

    for (int i = 0; i < max_iter; i++) {

        // create a ts message
        auto tp0 = chrono::time_point_cast<chrono::nanoseconds>(chrono::high_resolution_clock::now());
        long t0 = tp0.time_since_epoch().count();
        // NOTE: Either use local vars for oss and input_buff,
        //       or clear them before writing
        //oss << " " << i;
        oss.clear();
        oss << " 78.56 98.01\n";

        // send
        size_t n_write = pSerial->write(" 78.56 98.01\n");

        // receive
        input_buff.clear();
        size_t n_read = pSerial->readline(input_buff);


        // calculate round trip
        double rt = 0.0;
        if (n_read > 0) {
            auto tp1 = chrono::time_point_cast<chrono::nanoseconds>(chrono::high_resolution_clock::now());
            long t1 = tp1.time_since_epoch().count();
            rt = static_cast<double>(t1 - t0) * 1e-3;
        }

        // calculate the average
        avg_rt = (avg_rt * i + rt) / (i + 1) ;

        if (i % print_cnt == 0) {
            if (n_read > 0) {
                input_buff.back() = '0';
            }
            cout << "- sent: " << n_write << ", received: " << n_read << ": " << input_buff << ", curr avg: " << fixed << avg_rt << " (us)\n";
        }
    }

    cout << "** Average round-trip time: " << fixed << setprecision(6) << avg_rt << " (s)\n";
}


int main() {

    string port = "/dev/ttyACM0";
    int bd = 115200;
    auto timeout_ms = serial::Timeout::simpleTimeout(100);

    auto mpSerial = make_shared<serial::Serial>(port, bd, timeout_ms);

    test_speed(mpSerial);

    return 0;
}