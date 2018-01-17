#ifndef KOALA_DRIVER_JLG_JPEG_READER_H
#define KOALA_DRIVER_JLG_JPEG_READER_H

#define JLG_HEADER_SIZE 32

#include <cstring>
#include <iostream>
#include <iomanip>
#include <cstdio>

#include <jpeglib.h>
#include <jerror.h>
#include <boost/asio.hpp>

namespace kteams {
  namespace koala {
    void throw_invalid_header(unsigned char expected, unsigned char got);
    void checkheader(unsigned char * header, unsigned long int& width, unsigned long int& height, unsigned long int& size);

    void init_source(j_decompress_ptr p);
    boolean fill_input_buffer(j_decompress_ptr p);
    void skip_input_data(j_decompress_ptr p, long int l);
    boolean resync_to_restart(j_decompress_ptr p, int l);
    void term_source(j_decompress_ptr p);
    void error_exit(j_common_ptr p);
    void emit_message(j_common_ptr p, int l);
    void output_message(j_common_ptr p);
    void format_message(j_common_ptr p, char * s);
    void reset_error_mgr(j_common_ptr p);

    int jlg_decompress(unsigned char * inputbuffer, int width, int height, int size, int width_old, int height_old, unsigned char** decompressed_image, unsigned char ** jpeg_linebuffer);

    struct ConnectionParams {
      boost::asio::ip::tcp::iostream socket;
      std::string hostname;
      std::string port;

      unsigned char * header;
      unsigned long int width, height;
      unsigned long int size;
      unsigned long int width_old, height_old;
      unsigned long int size_old;
      unsigned char * inputbuffer;
      unsigned char * jpeg_linebuffer;
      unsigned char * decompressed_image;
    };

    void open_connection(std::string hostname, std::string port,
			 ConnectionParams& params);
    void close_connection(ConnectionParams& params);
    void read_image(ConnectionParams& params);
  }
}
#endif
