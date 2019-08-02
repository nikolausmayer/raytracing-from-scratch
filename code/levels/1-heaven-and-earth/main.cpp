
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>


class Vector {
public:
  /// Constructor
  Vector(float x=0.f, float y=0.f, float z=0.f)
    : x(x), y(y), z(z)
  { }

  float x;
  float y;
  float z;

  /// Copy constructor
  Vector(const Vector& rhs)
    : x(rhs.x), y(rhs.y), z(rhs.z)
  { }

  /// Addition
  Vector operator+(const Vector& rhs) const
  {
    return Vector{x+rhs.x,
                  y+rhs.y,
                  z+rhs.z};
  }

  /// Subtraction
  Vector operator-(const Vector& rhs) const
  {
    return *this + rhs*-1;
  }

  /// Negation
  Vector operator-() const
  {
    return *this * -1;
  }

  /// Linear scaling
  Vector operator*(float r) const
  {
    return Vector{x*r, y*r, z*r};
  }

  /// Dot product
  float operator%(const Vector& rhs) const
  {
    return x*rhs.x + y*rhs.y + z*rhs.z;
  }

  /// Cross product
  Vector operator^(const Vector& rhs) const
  {
    return Vector{y*rhs.z - z*rhs.y,
                  z*rhs.x - x*rhs.z,
                  x*rhs.y - y*rhs.x};
  }

  /// Vector length
  float length() const
  {
    return std::sqrt(x*x + y*y + z*z);
  }

  /// Normalization to unit length
  Vector operator!() const
  {
    return (*this)*(1.f/length());
  }
};



Vector get_ground_color(const Vector& ray_origin,
                        const Vector& ray_direction)
{
  //return {255, 0, 0};
  const float distance = -ray_origin.y / ray_direction.y;
  const float x = ray_origin.x + ray_direction.x * distance;
  const float z = ray_origin.z + ray_direction.z * distance;

  if ((int)std::abs(std::floor(x)) % 2 ==
      (int)std::abs(std::floor(z)) % 2 )
  {
    return {255, 0, 0};
  } else {
    return {255, 255, 255};
  }
}

Vector get_sky_color(const Vector& ray_direction)
{
  //return {0, 0, 255};
  return Vector{0.7, 0.6, 1.0}*255 * std::pow(1-ray_direction.y, 2);
}


int main() {
  /// LEFT-hand coordinate system
  /// Z = forwards
  const Vector Z{0, 0, 1};
  /// X = right
  const Vector X{0.002, 0, 0};
  /// Y = up
  const Vector Y{0, 0.002, 0};



  std::ofstream outfile("img.ppm", std::ios::binary);

  /// PPM Header
  outfile << "P6 512 512 255 ";

  for (int y = 256; y >= -255; --y) {
    std::cout << std::setw(3) << 100*(256-y+1)/512 << "%\r" << std::flush;
    for (int x = -255; x <= 256; ++x) {

      Vector color{0, 0, 0};

      Vector ray_origin{0, 1, -4};
      Vector ray_direction = !Vector{X*(x-0.5f) + 
                                     Y*(y-0.5f) +
                                     Z};

      if (ray_direction.y < 0) {
        color = get_ground_color(ray_origin, ray_direction);
      } else {
        color = get_sky_color(ray_direction);
      }


      outfile 
      << static_cast<unsigned char>(std::max(0.f, std::min(255.f, color.x)))
      << static_cast<unsigned char>(std::max(0.f, std::min(255.f, color.y)))
      << static_cast<unsigned char>(std::max(0.f, std::min(255.f, color.z)));
    }
  }

  ///
  return EXIT_SUCCESS;
}

