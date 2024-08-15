#ifndef DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP
#define DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP

#include <sstream>
#include <libserial/SerialPort.h>

#include <iostream>


LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  // Just handle some common baud rates
  switch (baud_rate)
  {
    case 1200: return LibSerial::BaudRate::BAUD_1200;
    case 1800: return LibSerial::BaudRate::BAUD_1800;
    case 2400: return LibSerial::BaudRate::BAUD_2400;
    case 4800: return LibSerial::BaudRate::BAUD_4800;
    case 9600: return LibSerial::BaudRate::BAUD_9600;
    case 19200: return LibSerial::BaudRate::BAUD_19200;
    case 38400: return LibSerial::BaudRate::BAUD_38400;
    case 57600: return LibSerial::BaudRate::BAUD_57600;
    case 115200: return LibSerial::BaudRate::BAUD_115200;
    case 230400: return LibSerial::BaudRate::BAUD_230400;
    default:
      std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << std::endl;
      return LibSerial::BaudRate::BAUD_57600;
  }
}

class ArduinoComms
{

public:

  ArduinoComms() = default;

  void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
  {  
    timeout_ms_ = timeout_ms;
    serial_conn_.Open(serial_device);
    serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
    serial_conn_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    serial_conn_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
    serial_conn_.SetParity(LibSerial::Parity::PARITY_NONE);
    serial_conn_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
  }


  void disconnect()
  {
    serial_conn_.Close();
  }


  bool connected() const
  {
    return serial_conn_.IsOpen();
  }


  std::string send_msg(const std::string &msg_to_send,bool read_res=false, bool print_output = false)
  {
    // serial_conn_.FlushIOBuffers(); // Just in case
    serial_conn_.Write(msg_to_send);
    serial_conn_.DrainWriteBuffer(); // wait till write finish

    std::string response = "";
    
    if (read_res)
    {
        try
      {
        // Responses end with \r\n so we will read up to (and including) the \n.
        serial_conn_.ReadLine(response, '\n', 25);
      }
      catch (const LibSerial::ReadTimeout&)
      {
          std::cerr << "The ReadByte() call has timed out." << std::endl ;
      }

      if (print_output)
      {
        std::cout << "Send: " << msg_to_send << " Recv: " << response << std::endl;
      }
    }

    return response;
  }


  void send_empty_msg()
  {
    std::string response = send_msg("\r");
  }


  /* read both encoder value and velocity */
  void read_states_values(double &left_vel, int &left_enc, double &right_vel, int &right_enc)
  {
    std::string response = send_msg("s\r", true, false);
    std::stringstream ss(response);
    
    std::string substr1, substr2, substr3, substr4;
    
    std::getline(ss, substr1, ' ');
    std::getline(ss, substr2, ' ');
    std::getline(ss, substr3, ' ');
    std::getline(ss, substr4, ' ');

    left_vel = std::atof(substr1.c_str()); // rpm
    left_enc = std::atoi(substr2.c_str());
    right_vel = std::atof(substr3.c_str());
    right_enc = std::atoi(substr4.c_str()); // rpm
  }


  /* only read velocity */
  void read_states_values2(double &left_vel, double &right_vel)
  {
    std::string response = send_msg("s\r");
    std::stringstream ss(response);
    
    std::string substr1, substr2;
    
    std::getline(ss, substr1, ' ');
    std::getline(ss, substr2, ' ');


    left_vel = std::atof(substr1.c_str()); // rpm
    right_vel = std::atof(substr2.c_str());
  }


  /* only set motor velocity */
  void set_motor_values(double val_1, double val_2)
  {
    std::stringstream ss;
    ss << "v " << val_1 << " " << val_2 << "\r";
    send_msg(ss.str());
  }


  /* set and read state value */
  void set_motor_values2(double val_1, double val_2, double &left_vel, double &right_vel)
  {
    std::stringstream ss;
    ss << "v " << val_1 << " " << val_2 << "\r";

    std::string response = send_msg(ss.str(), true);
    std::stringstream ss2(response);
    
    std::string substr1, substr2;
    
    std::getline(ss2, substr1, ' ');
    std::getline(ss2, substr2, ' ');

    left_vel = std::atof(substr1.c_str()); // rpm
    right_vel = std::atof(substr2.c_str());
  }


private:
    LibSerial::SerialPort serial_conn_;
    int timeout_ms_;
};

#endif // DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP