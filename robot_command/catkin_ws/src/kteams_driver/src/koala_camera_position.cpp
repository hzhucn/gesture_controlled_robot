
#include "koala_camera_position.h"

// Horizontal range
std::pair<int, int> kteams::koala::CameraPositionClient::pan_range = {-100, 100}; // deg
// Vertical range
std::pair<int, int> kteams::koala::CameraPositionClient::tilt_range = {0, 100}; // deg

kteams::koala::CameraPositionClient::CameraPositionClient() {}

int kteams::koala::CameraPositionClient::bound(int value, const std::pair<int, int>& range) {
  if(value < range.first)
    return range.first;
  else if(value > range.second)
    return range.second;
  else
    return value;
}

void kteams::koala::CameraPositionClient::connect(std::string hostname, std::string port) {
  socket.exceptions(std::ios::failbit | std::ios::badbit | std::ios::eofbit);
  socket.connect(hostname, port);

  log_input.open("input-camera-position.log");
  if(!log_input)
    std::cout << "Cannot open input log file." << std::endl;
      
  log_output.open("output-camera-position.log");
  if(!log_output)
    std::cout << "Cannot open output log file." << std::endl;

  writer.log(log_output,false);
  reader.log(log_input);

  // Let us use the tcp streams for the parsers
  writer.use(socket);
  reader.use(socket);
}


void kteams::koala::CameraPositionClient::close() {
  socket.close();
}

void kteams::koala::CameraPositionClient::read(int& pan, int& tilt)  {
  writer << 'H' << ktprot::flush;
  reader >> ktprot::next >> ktprot::expect('h');
  reader >> pan >> tilt;

}

void kteams::koala::CameraPositionClient::write(int pan, int tilt)  {    
  writer << 'C' << bound(pan, pan_range) << bound(tilt, tilt_range) << ktprot::flush;
  reader >> ktprot::next >> ktprot::expect('c');
}

