#include "koala_video.h"
#include <stdexcept>
#include <sstream>

void kteams::koala::throw_invalid_header(unsigned char expected, unsigned char got) {
  std::ostringstream ostr;
  ostr.str("");
  ostr << "Got invalid JLG JPEG header, expected [" << expected << "] but got [" << got << "]" ;
  throw std::logic_error(ostr.str());
}

void kteams::koala::checkheader(unsigned char * header, unsigned long int& width, unsigned long int& height, unsigned long int& size) {
  int i;

  if (header[0] == 0x10) {
    if (header[1] != 0x00) throw_invalid_header('\x00', header[1]);
    if (header[2] != 0x00) throw_invalid_header('\x00', header[2]);
    if (header[3] != 0x00) throw_invalid_header('\x00', header[3]);
    size = 0;
  } else if (header[0] == 0x20) {
    size = 0;
    for(i = 0; i < 3; i++) {
      size += header[1+i] << (i*8);
    }
  } else {
    std::ostringstream ostr;
    ostr.str("");
    ostr << "Got invalid JLG JPEG header, expected [" << '\x10' << "] or [" << '\x20' << " but got [" << int(header[0]) <<"]";
    throw std::logic_error(ostr.str());
  }

  for(unsigned int i = 4 ; i < 8; ++i) 
    if(header[i] != 0xFF) 
      throw_invalid_header('\xFF', header[i]);
  for(unsigned int i = 16 ; i < 20; ++i) 
    if(header[i] != 0x55) 
      throw_invalid_header('\x55', header[i]);
  for(unsigned int i = 28 ; i < 31; ++i) 
    if(header[i] != 0xAA) 
      throw_invalid_header('\xAA', header[i]);

  for(i = 0; i < 8; i++) 
    if (header[8+i] != header[20+i]) throw_invalid_header(header[8+i], header[20+i]);
  

  // The header is OK. Let's read the width and height :
  width = 0;
  height = 0;
  for(i = 0; i < 4; i++) {
    width += header[8+i] << (i*8);
    height += header[12+i] << (i*8);
  }
  if (header[0] == 0x10) {
    if (size == 0) size = 3 * (width) * (height);
  }
}


// Functions required by the jpeg decoder

void kteams::koala::init_source(j_decompress_ptr p) {
}

boolean kteams::koala::fill_input_buffer(j_decompress_ptr p) {
  return FALSE;
}

void kteams::koala::skip_input_data(j_decompress_ptr p, long int l) {
}

boolean kteams::koala::resync_to_restart(j_decompress_ptr p, int l) {
  return FALSE;
}

void kteams::koala::term_source(j_decompress_ptr p) {
}

void kteams::koala::error_exit(j_common_ptr p) {
}

void kteams::koala::emit_message(j_common_ptr p, int l) {
}

void kteams::koala::output_message(j_common_ptr p) {
}

void kteams::koala::format_message(j_common_ptr p, char * s) {
}

void kteams::koala::reset_error_mgr(j_common_ptr p) {
}

int kteams::koala::jlg_decompress(unsigned char * inputbuffer, int width, int height, int size, int width_old, int height_old, unsigned char** decompressed_image, unsigned char ** jpeg_linebuffer)
{
  struct jpeg_error_mgr jerr;
  struct jpeg_decompress_struct cinfo;
  struct jpeg_source_mgr source;
  int composantes;
  int i, j;

  cinfo.err = jpeg_std_error(&jerr);
  //cinfo.err = &jerr;
  memset(&jerr, sizeof(jerr), 0);
  jerr.error_exit = error_exit;
  jerr.emit_message = emit_message;
  jerr.output_message = output_message;
  jerr.format_message = format_message;
  jerr.reset_error_mgr = reset_error_mgr;

  // Décodage de l'image :
  jpeg_create_decompress(&cinfo);
  cinfo.src = &source;
  source.next_input_byte = inputbuffer;
  source.bytes_in_buffer = size;
  source.init_source = init_source;
  source.fill_input_buffer = fill_input_buffer;
  source.skip_input_data = skip_input_data;
  source.resync_to_restart = resync_to_restart;
  source.term_source = term_source;
  jpeg_read_header(&cinfo, TRUE);
  jpeg_start_decompress(&cinfo);
  if ((cinfo.output_width != width) || (cinfo.output_height != height)) {
    std::cerr << "Problem with the image size" << std::endl;
    goto ABANDON;
  }
  composantes = cinfo.num_components;
  if ((composantes != 1) && (composantes != 3)) {
    std::cerr << "Invalid number of components" << std::endl;
    goto ABANDON;
  }

  if((*decompressed_image) == NULL)
    {
      (*decompressed_image) = new unsigned char[3*width*height];
    }
  else if((width_old != width) || (height_old != height))
    {
      delete[] (*decompressed_image);
      (*decompressed_image) = new unsigned char[3*width*height];
    }


  if ((*jpeg_linebuffer) == NULL) {
    (*jpeg_linebuffer) = new JSAMPLE[composantes*width];
  } else if (width != width_old) {
    delete[] (*jpeg_linebuffer);
    (*jpeg_linebuffer) = new JSAMPLE[composantes*width];
  }


  for (j = 0; j < height; j++) {
    if (jpeg_read_scanlines(&cinfo, &(*jpeg_linebuffer), 1) == 0) {
      std::cerr << "Problem in the number of read lines while scanning jpeg, skipping the frame" << std::endl;;
      goto ABANDON;
    }
    for(i = 0; i < width; i++) {
      if (composantes == 1) {
	(*decompressed_image)[3*(i+j*width)+0] = (*jpeg_linebuffer)[i];
	(*decompressed_image)[3*(i+j*width)+1] = (*jpeg_linebuffer)[i];
	(*decompressed_image)[3*(i+j*width)+2] = (*jpeg_linebuffer)[i];
      } else if (composantes == 3) {
	(*decompressed_image)[3*(i+j*width)+0] = (*jpeg_linebuffer)[3*i+0];
	(*decompressed_image)[3*(i+j*width)+1] = (*jpeg_linebuffer)[3*i+1];
	(*decompressed_image)[3*(i+j*width)+2] = (*jpeg_linebuffer)[3*i+2];
      }
    }
  }
 ABANDON:
  jpeg_finish_decompress(&cinfo);
}


void kteams::koala::open_connection(std::string hostname, std::string port,
		     ConnectionParams& params) {
  params.hostname = hostname;
  params.port = port;
  
  params.header = new unsigned char[JLG_HEADER_SIZE];
  params.width = 0;
  params.height = 0;
  params.size = 0;
  params.width_old = 0;
  params.height_old = 0;
  params.size_old = 0;
  params.inputbuffer = NULL;
  params.jpeg_linebuffer = NULL;
  params.decompressed_image = NULL;

  params.socket.connect(params.hostname, params.port);
  
}

void kteams::koala::close_connection(ConnectionParams& params) {
  params.socket.close();

  delete[] params.inputbuffer;
  delete[] params.jpeg_linebuffer;
  delete[] params.decompressed_image;
  delete[] params.header;
}

void kteams::koala::read_image(ConnectionParams& params) {
  // Initialization of the header before receiving a frame
  for(unsigned int i = 0 ; i < JLG_HEADER_SIZE ; ++i) params.header[i] = 0;

  params.socket.read((char*)params.header, JLG_HEADER_SIZE);

  checkheader(params.header, params.width, params.height, params.size);
  int imagergb = (params.size == 3*params.width*params.height);
  if(imagergb)
    throw std::logic_error("I was supposed to decompress a JPEG buffer, but got a raw image");

  if(params.inputbuffer == NULL)
    params.inputbuffer = new unsigned char[params.size];
  else if(params.size_old < params.size) {
    delete[] params.inputbuffer;
    params.inputbuffer = new unsigned char[params.size];
  }
  
  // let us read the image data
  params.socket.read((char*)params.inputbuffer, params.size);

  // And decompress the jpeg image
  jlg_decompress(params.inputbuffer, params.width, params.height, params.size, params.width_old, params.height_old, &params.decompressed_image, &params.jpeg_linebuffer);
  
  params.width_old = params.width;
  params.height_old = params.height;
  params.size_old = params.size;
}
