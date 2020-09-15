#include <iostream>
#include <boost/asio.hpp>
#include <thread>
#include <chrono>
#include "my_stepper.h"

using namespace boost::asio;
using ip::tcp;
using std::string;
using std::cout;
using std::endl;

boost::asio::io_context io;
boost::asio::serial_port sp(io,"COM2");
tcp::socket socket1(io);
boost::system::error_code error1;
MyStepper stepper;

void serailSetP() {
    sp.write_some(buffer("1"));
//    std::cout<<"step: "<<stepper.step<<endl;
//    std::this_thread::sleep_for(std::chrono::milliseconds (1));
}

void serailSetN() {
    sp.write_some(buffer("2"));
//    std::cout<<"step: "<<stepper.step<<endl;
//    std::this_thread::sleep_for(std::chrono::milliseconds (1));
}

void socketSetP() {
    std::cout<<"socketSetP"<<endl;

    socket1.write_some(buffer("1"));
    std::this_thread::sleep_for(std::chrono::milliseconds (1));
}

void socketSetN() {
    std::cout<<"socketSetN"<<endl;
    socket1.write_some(buffer("2"));
    std::this_thread::sleep_for(std::chrono::milliseconds (1));
}

void socket_init() {
    //connection
    socket1.connect( tcp::endpoint( boost::asio::ip::address::from_string("127.0.0.1"), 8234 ));
}

void stepper_init() {
    stepper.stepN = serailSetN;
    stepper.stepP = serailSetP;
}

void serial_init() {
    sp.set_option(boost::asio::serial_port::baud_rate(115200));
    sp.set_option(boost::asio::serial_port::flow_control());
    sp.set_option(boost::asio::serial_port::parity());
    sp.set_option(boost::asio::serial_port::stop_bits());
    sp.set_option(boost::asio::serial_port::character_size(8));
}

int main() {

    serial_init();
    stepper_init();

    stepper.v = 0x8000;

    while (true) {
        stepper.run();
        std::this_thread::sleep_for(std::chrono::microseconds (1000));
//        unsigned __int64 now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
//        std::cout << now << std::endl;
    }


    return 0;
}