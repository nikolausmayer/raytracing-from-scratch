
#include <fstream>


int main() {
  std::ofstream outfile("img.ppm", std::ios::binary);

  /// PPM Header
  outfile << "P6 512 512 255 ";

  for (int y = 0; y < 512; ++y) {
    for (int x = 0; x < 512; ++x) {
      outfile << static_cast<unsigned char>(255)
              << static_cast<unsigned char>(0)
              << static_cast<unsigned char>(0);
    }
  }

  return EXIT_SUCCESS;
}

