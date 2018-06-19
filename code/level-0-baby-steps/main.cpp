/**
 * Author: Nikolaus Mayer, 2018 (mayern@cs.uni-freiburg.de)
 */

#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <vector>



int main(){
  std::ofstream outfile("img.ppm");

  /// PPM header 
  /// The magic number "P6" means "color PPM in binary format (= not ASCII)"
  outfile << "P6 512 512 255 ";

  for(int y = 256; y >= -255; --y) {   //For each row
    for(int x = -255; x <= 256; ++x) {   //For each pixel in a row

      /// Write this pixel's RGB color triple (each a single byte)
      outfile << (unsigned char)255
              << (unsigned char)0
              << (unsigned char)0;

    }
  }

  /// Image done
  outfile.close();
}


