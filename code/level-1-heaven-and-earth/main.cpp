/**
 * Author: Nikolaus Mayer, 2018 (mayern@cs.uni-freiburg.de)
 */

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>


/// Define a vector class with constructor and operator
struct Vector
{
  /// Constructor
  Vector(float a=0.f, float b=0.f, float c=0.f)
    : x(a), y(b), z(c)
  { }

  /// Copy constructor
  Vector(const Vector& rhs)
    : x(rhs.x), y(rhs.y), z(rhs.z)
  { }

  /// Vector has three float attributes
  /// 
  union{float x; float r;};
  union{float y; float g;};
  union{float z; float b;};

  /// Vector addition
  Vector operator+(const Vector& r) const
  {
    return Vector{x+r.x,y+r.y,z+r.z};
  }

  /// Subtraction
  Vector operator-(const Vector& r) const
  {
    return *this + r*-1;
  }

  /// Unary minus / negation ("-vector")
  Vector operator-() const
  {
    return *this * -1;
  }

  /// Linear scaling
  Vector operator*(float r) const
  {
    return Vector{x*r,y*r,z*r};
  }

  /// Dot product with another Vector
  float operator%(Vector r) const
  {
    return x*r.x+y*r.y+z*r.z;
  }

  /// Cross-product with another Vector
  Vector operator^(Vector r) const
  {
    return Vector{y*r.z-z*r.y,
                  z*r.x-x*r.z,
                  x*r.y-y*r.x};
  }

  /// Vector length
  float length() const
  {
    return std::sqrt(*this % *this);
  }

  /// Normalization to unit length
  Vector operator!() const
  {
    return (*this)*(1./length());
  }
};


Vector get_ground_color(const Vector& ray_origin,
                        const Vector& ray_direction)
{
  //return {255,0,0};
  float distance = -ray_origin.y/ray_direction.y;
  float floor_hit_x = ray_origin.x + ray_direction.x*distance;
  float floor_hit_z = ray_origin.z + ray_direction.z*distance;
  if ((int)std::abs(std::floor(floor_hit_x))%2 == 
      (int)std::abs(std::floor(floor_hit_z))%2) 
  {
    return {255,0,0};
  } else {
    return {255,255,255};
  }
}


Vector get_sky_color(const Vector& ray_direction)
{
  //return {0,0,255};
  return Vector{.7,.6,1}*255*std::pow(1-ray_direction.y,2);
}


int main(){

  /// LEFT-HANDED COORDINATE SYSTEM!
  /// FORWARDS vector (viewing direction)
  Vector ahead{0,0,1};
  /// RIGHT vector
  Vector right{.002,0,0};
  /// UP vector
  Vector up{0,.002,0};

  std::ofstream outfile("img.ppm");


  /// PPM header 
  /// The magic number "P6" means "color PPM in binary format (= not ASCII)"
  outfile << "P6 512 512 255 ";

  for(int y = 256; y >= -255; --y) {   //For each row
    for(int x = -255; x <= 256; ++x) {   //For each pixel in a row

      Vector color{0,0,0};

      Vector ray_origin{0,1,-4};
      Vector ray_direction = !Vector{right*(x-0.5) + up*(y-0.5) + ahead};

      if (ray_direction.y < 0) {
        color = get_ground_color(ray_origin, ray_direction);
      } else {
        color = get_sky_color(ray_direction);
      }

      /// Write this pixel's RGB color triple (each a single byte)
      outfile << (unsigned char)(std::max(0.f, std::min(color.r, 255.f)))
              << (unsigned char)(std::max(0.f, std::min(color.g, 255.f)))
              << (unsigned char)(std::max(0.f, std::min(color.b, 255.f)));

    }
  }

  /// Image done
  outfile.close();

  /// Bye!
  return EXIT_SUCCESS;
}


