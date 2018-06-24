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
    return Vector(x+r.x,y+r.y,z+r.z);
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
    return Vector(x*r,y*r,z*r);
  }

  /// Dot product with another Vector
  float operator%(Vector r) const
  {
    return x*r.x+y*r.y+z*r.z;
  }

  /// Cross-product with another Vector
  Vector operator^(Vector r) const
  {
    return Vector(y*r.z-z*r.y,
                  z*r.x-x*r.z,
                  x*r.y-y*r.x);
  }

  /// Normalization to unit length
  Vector operator!() const
  {
    return (*this)*(1./sqrt(*this % *this));
  }
};


/// Abstract "raytraceable object" superclass
struct Object
{
  Object()
    : reflectivity(1),
      color({255,255,255})
  { }

  /// Check if object is hit by ray, and if yes, compute next ray
  ///
  /// incoming_ray_origin:    Origin 3D point of the incoming ray
  /// incoming_ray_direction: Direction vector of the incoming ray
  /// outgoing_ray_origin:    Origin 3D point of the reflected ray; i.e. the
  ///                         ray/object intersection/hit point
  /// outgoing_ray_direction: Direction vector of the outgoing/reflected ray
  /// hit_distance:           Distance the incoming ray has traveled from its
  ///                         origin to the hit point
  /// hit_color:              Object surface color at hit point
  /// object_reflectivity:    Ratio of incoming ray's energy reflected into the
  ///                         outgoing ray; rest is "absorbed" by the object
  virtual bool is_hit_by_ray(const Vector& incoming_ray_origin,
                             const Vector& incoming_ray_direction,
                             Vector& outgoing_ray_origin,
                             Vector& outgoing_ray_direction,
                             float& hit_distance,
                             Vector& hit_color,
                             float& object_reflectivity) const = 0;

  void set_reflectivity(float v)
  {
    reflectivity = v;
  };

  void set_color(const Vector& v)
  {
    color = v;
  };

  float reflectivity;
  Vector color;
};


/// A traceable triangle (base polygon); defined by 3 3D points
struct Triangle : Object
{
  Triangle(const Vector& p0,
           const Vector& p1,
           const Vector& p2)
    : p0(p0), p1(p1), p2(p2)
  {
    /// "u" and "v" vectors define the triangle (together with p0)
    ///
    ///       p1 
    ///       O   ^
    ///      /|   |
    ///     //|   | u
    ///    ///|   |
    ///   ////|   |
    ///  O----O   x
    ///  p2   p0
    ///
    ///  <----x
    ///    v
    ///
    u = p1-p0;
    v = p2-p0;
    normal = (u^v);
  }

  Vector p0, p1, p2;
  Vector u, v, normal;

  bool is_hit_by_ray(const Vector& incoming_ray_origin,
                     const Vector& incoming_ray_direction,
                     Vector& outgoing_ray_origin,
                     Vector& outgoing_ray_direction,
                     float& hit_distance,
                     Vector& hit_color,
                     float& object_reflectivity) const
  {
    hit_distance = (-normal%(incoming_ray_origin-p0)) / 
                   (-incoming_ray_direction%-normal);
    /// Without a little slack, a reflected ray sometimes hits the same
    /// object again (machine precision..)
    if (hit_distance <= 1e-6)
      return false;

    float u_factor = (u^-incoming_ray_direction)%(incoming_ray_origin-p0) /
                     (-incoming_ray_direction%-normal);
    float v_factor = (-incoming_ray_direction^v)%(incoming_ray_origin-p0) /
                     (-incoming_ray_direction%-normal);
    if (u_factor < 0 or
        v_factor < 0 or
        u_factor+v_factor > 1)
      return false;

    /// Temporary normal vector
    const Vector n = normal;

    outgoing_ray_origin = p0+v*u_factor+u*v_factor;
    outgoing_ray_direction = !(incoming_ray_direction - !n*(incoming_ray_direction%!n)*2);
    hit_color = color;
    object_reflectivity = reflectivity;
    return true;
  }
};


Vector get_ground_color(const Vector& ray_origin,
                        const Vector& ray_direction)
{
  float distance = std::abs(ray_origin.y/ray_direction.y);
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
  return Vector(.7,.6,1)*255*pow(1-ray_direction.y,2);
}


int main(){

  /// FORWARDS vector (viewing direction)
  Vector ahead(0,0,1);
  /// RIGHT vector
  Vector right(.002,0,0);
  /// DOWN vector
  Vector down(0,.002,0);

  std::ofstream outfile("img.ppm");


  std::vector<Object*> scene_objects;

  scene_objects.push_back(new Triangle({-2,0,-1},
                                       {2,0,-1},
                                       {0,3,-1}));
  scene_objects.back()->set_color({0,0,255});
  scene_objects.back()->set_reflectivity(0.8);
  scene_objects.push_back(new Triangle({-2,0,-5},
                                       {2,0,-5},
                                       {0,3,-5}));
  scene_objects.back()->set_color({0,255,0});
  scene_objects.back()->set_reflectivity(0.8);


  const int max_hit_bounces{100};


  /// PPM header 
  /// The magic number "P6" means "color PPM in binary format (= not ASCII)"
  outfile << "P6 512 512 255 ";

  for(int y = 256; y >= -255; --y) {   //For each row
    for(int x = -255; x <= 256; ++x) {   //For each pixel in a row

      Vector color{0,0,0};

      Vector ray_origin{0,1,-4};
      Vector ray_direction = !Vector{right*(x-0.5) + down*(y-0.5) + ahead};

      Vector ray_hit_at, 
             ray_bounced_direction, 
             hit_color;
      float distance_to_hit,
            reflectivity_at_hit, 
            ray_energy_left=1;

      for (int bounce = 0; bounce <= max_hit_bounces; ++bounce) {
        /// Compute object intersections
        bool an_object_was_hit{false};
        float min_hit_distance{std::numeric_limits<float>::max()};
        Object* closest_object_ptr{nullptr};
        for (const auto& object : scene_objects) {
          if (object->is_hit_by_ray(ray_origin, ray_direction,
                                    ray_hit_at, ray_bounced_direction,
                                    distance_to_hit,
                                    hit_color,
                                    reflectivity_at_hit))
          {
            an_object_was_hit = true;

            if (distance_to_hit < min_hit_distance) {
              min_hit_distance = distance_to_hit;
              closest_object_ptr = object;
            }
          }
        }
        /// Compute color of hit object/ground/sky
        if (an_object_was_hit) {
          closest_object_ptr->is_hit_by_ray(ray_origin, ray_direction,
                                            ray_hit_at, ray_bounced_direction,
                                            distance_to_hit,
                                            hit_color,
                                            reflectivity_at_hit);
          ray_origin = ray_hit_at;
          ray_direction = ray_bounced_direction;
        } else {
          if (ray_direction.y < 0) {
            hit_color = get_ground_color(ray_origin, ray_direction);
            reflectivity_at_hit = 0;
          } else {
            hit_color = get_sky_color(ray_direction);
            reflectivity_at_hit = 0;
          }
        }

        color = color + (hit_color*(ray_energy_left*(1-reflectivity_at_hit)));
        ray_energy_left *= reflectivity_at_hit;
        if (ray_energy_left <= 0)
          break;
      }


      /// Write this pixel's RGB color triple (each a single byte)
      outfile << (unsigned char)(std::max(0.f, std::min(color.r, 255.f)))
              << (unsigned char)(std::max(0.f, std::min(color.g, 255.f)))
              << (unsigned char)(std::max(0.f, std::min(color.b, 255.f)));

    }
  }

  /// Image done
  outfile.close();
}


