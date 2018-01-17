#pragma once

// Definition of the communication with the camera position server

#include <ktprot.h>
#include <boost/asio.hpp>
#include <fstream>
#include <tuple>

namespace kteams {
  namespace koala {

    // The IR and the wheels are accessed through the wheel server
    // and the communication is handled by ktprot
    struct CameraPositionClient {
    
      // Horizontal range
      static std::pair<int, int> pan_range;
      // Vertical range
      static std::pair<int, int> tilt_range;

      ktprot::Writer writer;
      ktprot::Reader reader;
      boost::asio::ip::tcp::iostream socket;
      std::ofstream log_input;
      std::ofstream log_output;

      CameraPositionClient();

      static int bound(int value, const std::pair<int, int>& range);
      void connect(std::string hostname, std::string port);
      void close();
      void read(int& pan, int& tilt);
      void write(int pan, int tilt);
    };

  }
}
