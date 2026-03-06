// send_once.cpp
#include "../include/rosmaster_driver_diff/rosmaster.hpp"
#include <boost/asio.hpp>
#include <iostream>

using namespace rosmaster;

int main(int argc, char** argv){
  std::string dev = "/dev/ttyUSB0";
  unsigned int baud = 115200;
  if (argc>1) dev = argv[1];
  if (argc>2) baud = std::stoul(argv[2]);

  boost::asio::io_service io;
  boost::asio::serial_port port(io);
  port.open(dev);
  port.set_option(boost::asio::serial_port_base::baud_rate(baud));

  // Example: front-left 50, front-right -50, back-left = follow = 50, back-right = -50
  auto frame = build_set_motor(50, -50, 50, -50);
  boost::asio::write(port, boost::asio::buffer(frame.data(), frame.size()));
  std::cout << "Sent: " << to_hex(frame) << std::endl;

  port.close();
  return 0;
}