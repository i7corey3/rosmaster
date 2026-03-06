#ifdef ROSMASTER_CONTROLLER_HPP_
#define ROSMASTER_CONTROLLER_HPP_


using namespace rosmaster;

// Read



// std::cout << "Rosmaster frame helper demo\n";

//   // Example 1: set_motor(50, -50, 0, 127)
//   auto f1 = build_set_motor(50, -50, 0, 127);
//   std::cout << "set_motor(50,-50,0,127): " << to_hex(f1) << "\n";
//   // Expected (matches earlier Python example): FF FC 07 10 32 CE 00 7F 96

//   // Example 2: set_beep(50)
//   auto f2 = build_set_beep(50);
//   std::cout << "set_beep(50): " << to_hex(f2) << "\n";
//   // Example 3: set_car_run(forward=1, speed=50)
//   auto f3 = build_set_car_run(1, 1, 50, false);
//   std::cout << "set_car_run(forward,50): " << to_hex(f3) << "\n";

//   // Example 4: set_car_motion(0.5, 0, 0)
//   auto f4 = build_set_car_motion(0.5, 0.0, 0.0, 1);
//   std::cout << "set_car_motion(0.5,0,0): " << to_hex(f4) << "\n";

//   // Example 5: request encoders
//   auto f5 = build_request_data(0x0D, 0x00);
//   std::cout << "request encoders: " << to_hex(f5) << "\n";
//   // Expected: FF FC 05 50 0D 00 62

//   // Example 6: set_uart_servo(1,1500,500)
//   auto f6 = build_set_uart_servo(1, 1500, 500);
//   std::cout << "set_uart_servo(1,1500,500): " << to_hex(f6) << "\n";

//   // Example 7: set_pid_param(0.5,0.1,0.3)
//   auto f7 = build_set_pid_param(0.5, 0.1, 0.3, false);
//   std::cout << "set_pid_param(0.5,0.1,0.3): " << to_hex(f7) << "\n";


#endif // ROSMASTER_CONTROLLER_HPP_