#include "koala_wheels.h"
#include <limits>

kteams::koala::WheelServer::WheelServer() {}

void kteams::koala::WheelServer::connect(std::string hostname, std::string port) {
  socket.exceptions(std::ios::failbit | std::ios::badbit | std::ios::eofbit);
  socket.connect(hostname, port);

  log_input_wheels.open("input-wheels.log");
  if(!log_input_wheels)
    std::cout << "Cannot open input log file." << std::endl;
      
  log_output_wheels.open("output-wheels.log");
  if(!log_output_wheels)
    std::cout << "Cannot open output log file." << std::endl;

  writer.log(log_output_wheels,false);
  reader.log(log_input_wheels);

  // Let us use the tcp streams for the parsers
  writer.use(socket);
  reader.use(socket);

  
}

void kteams::koala::WheelServer::close() {
  socket.close();
}


kteams::koala::IRReader::IRReader(WheelServer& serv): server(serv) {}

double kteams::koala::IRReader::get_dist_from_ir(int ir_value)
{
  if(ir_value >= max_ir_value)
    return 0.0;
  else
    {
      double alpha = (ir_half_dist - ir_min_dist) / (exp(-log(0.5)/ir_order) - 1.0);
      double d = (exp(1.0/ir_order * log(max_ir_value/ir_value))-1.0) * alpha + ir_min_dist;
      if(d >= ir_max_dist)
	return std::numeric_limits<double>::infinity();
      else 
	return d;
    }
}

void kteams::koala::IRReader::read(std::array<double,nb_ir_sensors>& ranges, std::array<int, nb_ir_sensors>& raw) {
  server.writer << 'N' << ktprot::flush;
  server.reader >> ktprot::next >> ktprot::expect('n');
  for(unsigned int i=0;i<16;++i)
      server.reader >> raw[i];

  int tmp;
  for(unsigned int i = 0 ; i < 4 ; ++i) {
      tmp = raw[i];
      raw[i] = raw[7 - i];
      raw[7 - i] = tmp;
    }

  for(unsigned int i = 0 ; i < nb_ir_sensors ; ++i)
      ranges[i] = get_dist_from_ir(raw[i]);
}

kteams::koala::WheelSpeed::WheelSpeed(WheelServer& serv): server(serv) {}

void kteams::koala::WheelSpeed::read(double& linear, double& angular) const {
  server.writer << 'E' << ktprot::flush;
  server.reader >> ktprot::next >> ktprot::expect('e');
  
  // We extract the linear velocities of the two wheels
  int left_wheel_pulses, right_wheel_pulses; // 
  server.reader >> left_wheel_pulses;
  server.reader >> right_wheel_pulses;

  double left_wheel_linear_speed = factor_pulse_to_speed * left_wheel_pulses; // in m.s-1
  double right_wheel_linear_speed = factor_pulse_to_speed * right_wheel_pulses; // in m.s-1

  linear = (left_wheel_linear_speed + right_wheel_linear_speed)/2.0;
  angular = (right_wheel_linear_speed - left_wheel_linear_speed) / inter_wheels_distance;
}



void kteams::koala::WheelSpeed::write(double linear, double angular) const {
    // The koala is moving with a differential drive
    double Lw = inter_wheels_distance * angular;

    double left_wheel_linear_speed = (2.0 * linear - Lw) / 2.0;
    double right_wheel_linear_speed = (2.0 * linear + Lw) / 2.0;

    int left_wheel_pulses = int(left_wheel_linear_speed / (factor_pulse_to_speed));
    int right_wheel_pulses = int(right_wheel_linear_speed / (factor_pulse_to_speed));

    server.writer << 'D' << left_wheel_pulses << right_wheel_pulses << ktprot::flush;
    server.reader >> ktprot::next >> ktprot::expect('d');
}
