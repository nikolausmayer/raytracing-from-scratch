/**
 * Author: Nikolaus Mayer, 2018 (mayern@cs.uni-freiburg.de)
 */

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <vector>

//
/// Sample from a uniform random distribution in [0, 1]
float uniform_random_01()
{
  /// RAND_MAX in <cstdlib>
  return static_cast<float>(rand())/RAND_MAX - 0.5f;
}


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


///
/// A RotationMatrix implements a 3x3 matrix in the SO(3) group. It holds that:
/// - a RotationMatrix multiplied with its transposed yields identity
///     R * R^T = I
/// - the transposed of a RotationMatrix is its inverse
///     R^-1 = R^T
///
struct RotationMatrix
{
   /// Constructor using 3 angles, Euler sequence XYZ with static axes
   RotationMatrix(float x, float y, float z)
   {
     *this = RotationMatrix(1.0f, 0.0f, 0.0f,
                            0.0f, std::cos(x), -std::sin(x),
                            0.0f, std::sin(x), std::cos(x)) *
             RotationMatrix(std::cos(y), 0.0f, std::sin(y),
                            0.0f, 1.0f, 0.0f,
                            -std::sin(y), 0.0f, std::cos(y)) *
             RotationMatrix(std::cos(z), -std::sin(z), 0.0f,
                            std::sin(z), std::cos(z), 0.0f,
                            0.0f, 0.0f, 1.0f);
   }

   /// Empty constructor
   //RotationMatrix()
   //{
   //  *this = RotationMatrix(0,0,0,0,0,0,0,0,0);
   //}

   /// Constructor using 9 explicit values
   RotationMatrix(float d00, float d01, float d02,
                  float d10, float d11, float d12,
                  float d20, float d21, float d22)
   {
     data[0][0] = d00; data[0][1] = d01; data[0][2] = d02;
     data[1][0] = d10; data[1][1] = d11; data[1][2] = d12;
     data[2][0] = d20; data[2][1] = d21; data[2][2] = d22;
   }

   /// Matrix-Matrix multiplication
   RotationMatrix operator*(const RotationMatrix& rhs) const
   {
     RotationMatrix result(0,0,0,0,0,0,0,0,0);
     for (size_t y = 0; y < 3; ++y) {
      for (size_t x = 0; x < 3; ++x) {
        result.data[y][x] = data[y][0]*rhs.data[0][x] +
                            data[y][1]*rhs.data[1][x] +
                            data[y][2]*rhs.data[2][x];
      }
     }
     return result;
   }

   /// Matrix-Vector multiplication
   Vector operator*(const Vector& rhs) const
   {
     return Vector(data[0][0]*rhs.x + data[0][1]*rhs.y + data[0][2]*rhs.z,
                   data[1][0]*rhs.x + data[1][1]*rhs.y + data[1][2]*rhs.z,
                   data[2][0]*rhs.x + data[2][1]*rhs.y + data[2][2]*rhs.z);
   }

   float data[3][3];
};


/// Abstract "raytraceable object" superclass
struct Object
{
  Object()
    : roughness(0),
      diffuse_factor(1),
      specular_factor(1),
      reflectivity(1),
      color(255,255,255)
  { }

  /// Check if object is hit by ray, and if yes, compute next ray
  ///
  /// incoming_ray_origin:    Origin 3D point of the incoming ray
  /// incoming_ray_direction: Direction vector of the incoming ray
  /// outgoing_ray_origin:    Origin 3D point of the reflected ray; i.e. the
  ///                         ray/object intersection/hit point
  /// outgoing_ray_direction: Direction vector of the outgoing/reflected ray
  /// outgoing_normal:        Object surface's normal vector at hit point
  /// hit_distance:           Distance the incoming ray has traveled from its
  ///                         origin to the hit point
  /// hit_color:              Object surface color at hit point
  /// object_reflectivity:    Ratio of incoming ray's energy reflected into the
  ///                         outgoing ray; rest is "absorbed" by the object
  virtual bool is_hit_by_ray(const Vector& incoming_ray_origin,
                             const Vector& incoming_ray_direction,
                             Vector& outgoing_ray_origin,
                             Vector& outgoing_ray_direction,
                             Vector& outgoing_normal,
                             float& hit_distance,
                             Vector& hit_color,
                             float& object_diffuse_factor,
                             float& object_specular_factor,
                             float& object_reflectivity) const = 0;

  void set_roughness(float v)
  {
    roughness = v;
  };

  void set_diffuse_factor(float v)
  {
    diffuse_factor = v;
  };

  void set_specular_factor(float v)
  {
    specular_factor = v;
  };

  void set_reflectivity(float v)
  {
    reflectivity = v;
  };

  void set_color(const Vector& v)
  {
    color = v;
  };

  float roughness;
  float diffuse_factor;
  float specular_factor;
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
    normal = (v^u);
  }

  Vector p0, p1, p2;
  Vector u, v, normal;

  bool is_hit_by_ray(const Vector& incoming_ray_origin,
                     const Vector& incoming_ray_direction,
                     Vector& outgoing_ray_origin,
                     Vector& outgoing_ray_direction,
                     Vector& outgoing_normal,
                     float& hit_distance,
                     Vector& hit_color,
                     float& object_diffuse_factor,
                     float& object_specular_factor,
                     float& object_reflectivity) const
  {
    if (!normal%-!incoming_ray_direction < 0)
      return false;

    const float& pox{p0.x};
    const float& poy{p0.y};
    const float& poz{p0.z};
    const float& ux{u.x};
    const float& uy{u.y};
    const float& uz{u.z};
    const float& vx{v.x};
    const float& vy{v.y};
    const float& vz{v.z};
    const float& rx{incoming_ray_direction.x};
    const float& ry{incoming_ray_direction.y};
    const float& rz{incoming_ray_direction.z};
    const float& ox{incoming_ray_origin.x};
    const float& oy{incoming_ray_origin.y};
    const float& oz{incoming_ray_origin.z};
    const float u_factor = (-(ox - pox)*(ry*vz - rz*vy) + (oy - poy)*(rx*vz - rz*vx) - (oz - poz)*(rx*vy - ry*vx))/(rx*uy*vz - rx*uz*vy - ry*ux*vz + ry*uz*vx + rz*ux*vy - rz*uy*vx);
    const float v_factor = ((ox - pox)*(ry*uz - rz*uy) - (oy - poy)*(rx*uz - rz*ux) + (oz - poz)*(rx*uy - ry*ux))/(rx*uy*vz - rx*uz*vy - ry*ux*vz + ry*uz*vx + rz*ux*vy - rz*uy*vx);
    const float ray_factor = (-(ox - pox)*(uy*vz - uz*vy) + (oy - poy)*(ux*vz - uz*vx) - (oz - poz)*(ux*vy - uy*vx))/(rx*uy*vz - rx*uz*vy - ry*ux*vz + ry*uz*vx + rz*ux*vy - rz*uy*vx);

    if (u_factor < 0 or
        v_factor < 0 or
        u_factor+v_factor > 1 or
        ray_factor < 0)
      return false;
    hit_distance = (incoming_ray_direction*ray_factor).length();
    if (hit_distance <= 1e-6)
      return false;

    outgoing_ray_direction = !(incoming_ray_direction + !normal*(-incoming_ray_direction%!normal)*2);
    outgoing_ray_direction = !(outgoing_ray_direction + !Vector(uniform_random_01(),
                                                                uniform_random_01(),
                                                                uniform_random_01())*roughness);
    if (!normal%outgoing_ray_direction <= 0) {
      outgoing_ray_direction = !(outgoing_ray_direction + (normal%-outgoing_ray_direction)*2);
    }
    
    outgoing_ray_origin = p0+u*u_factor+v*v_factor;
    outgoing_normal = !(outgoing_ray_direction-incoming_ray_direction);
    hit_color = color;
    object_diffuse_factor = diffuse_factor;
    object_specular_factor = specular_factor;
    object_reflectivity = reflectivity;
    return true;
  }
};


struct Sphere : Object
{
  Sphere(const Vector& center,
         float radius)
    : center(center), radius(radius)
  { }

  const Vector center;
  const float radius;

  bool is_hit_by_ray(const Vector& incoming_ray_origin,
                     const Vector& incoming_ray_direction,
                     Vector& outgoing_ray_origin,
                     Vector& outgoing_ray_direction,
                     Vector& outgoing_normal,
                     float& hit_distance,
                     Vector& hit_color,
                     float& object_diffuse_factor,
                     float& object_specular_factor,
                     float& object_reflectivity) const
  {
    const Vector p = incoming_ray_origin-center;
    const float b = p%incoming_ray_direction;
    const float c = p%p-radius*radius;

    //Does the ray hit the sphere ?
    if (b*b-c > 0) {
      //It does, compute the distance camera-sphere
      float s=-b-sqrt(radius*radius-p%p+b*b);

      if (s < 1e-3)
        return false;

      hit_distance=s;
      outgoing_ray_origin = incoming_ray_origin + incoming_ray_direction*hit_distance;
      const Vector normal = !(p+incoming_ray_direction*hit_distance);
      
      outgoing_ray_direction = !(incoming_ray_direction + !normal*(-incoming_ray_direction%!normal)*2);
      outgoing_ray_direction = !(outgoing_ray_direction + !Vector(uniform_random_01(),
                                                                  uniform_random_01(),
                                                                  uniform_random_01())*roughness);
      if (!normal%outgoing_ray_direction <= 0) {
        outgoing_ray_direction = !(outgoing_ray_direction + !normal*(normal%-outgoing_ray_direction)*2);
      }

      outgoing_normal = !(outgoing_ray_direction-incoming_ray_direction);

      hit_color = color;
      object_diffuse_factor = diffuse_factor;
      object_specular_factor = specular_factor;
      object_reflectivity = reflectivity;
      
      return true;
    } else {
      return false;
    }
  }
};



Vector get_ground_color(const Vector& ray_origin,
                        const Vector& ray_direction,
                        Vector& ray_hit_at)
{
  float distance = std::abs(ray_origin.y/ray_direction.y);
  ray_hit_at = ray_origin + ray_direction*distance;

  static unsigned char* texture_data{nullptr};
  static int tex_w{600};
  static int tex_h{400};
  if (not texture_data) {
    std::ifstream texture("texture.ppm");
    texture_data = new unsigned char[tex_w*tex_h*3];
    texture.read(reinterpret_cast<char*>(texture_data), 15);
    texture.read(reinterpret_cast<char*>(texture_data), tex_w*tex_h*3);
  }

  int tex_u = std::abs((int)(ray_hit_at.x*100)+1000)%tex_w;
  int tex_v = std::abs((int)(ray_hit_at.z*100)+1100)%tex_h;
  size_t color_start_idx = (tex_v*tex_w+tex_u)*3;
  return Vector((int)(texture_data[color_start_idx+0]),
                (int)(texture_data[color_start_idx+1]),
                (int)(texture_data[color_start_idx+2]));
}


Vector get_sky_color(const Vector& ray_direction)
{
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

  for (int frame_id = 0; frame_id < 200; ++frame_id) {

    std::ostringstream oss;
    oss << "frame_" << std::setw(4) << std::setfill('0') << frame_id << ".ppm";
    std::ofstream outfile(oss.str().c_str());
    std::cout << oss.str() << std::endl;


    std::vector<Object*> scene_objects;

    /// 2 spheres
    /// The 2 spheres have the same rotation element
    RotationMatrix s_rot{0,-frame_id/100.f*(22/7.f),0};
    scene_objects.push_back(new Sphere{s_rot*Vector{1,2,0}, .5});
    scene_objects.back()->set_roughness(0.75);
    scene_objects.back()->set_diffuse_factor(0);
    scene_objects.back()->set_specular_factor(0);
    scene_objects.back()->set_reflectivity(0.95);
    scene_objects.back()->set_color({255,255,255});
    scene_objects.push_back(new Sphere{s_rot*Vector{-1.25,.8,0}, .25});
    scene_objects.back()->set_diffuse_factor(0.9);
    scene_objects.back()->set_specular_factor(1);
    scene_objects.back()->set_reflectivity(0.05);
    scene_objects.back()->set_color({255,165,0});

    /// The octahedron has a separate rotation
    RotationMatrix o_rot{0.5f*frame_id/100.f*(22/7.f),
                         0.5f*frame_id/100.f*(22/7.f),
                         0.5f*frame_id/100.f*(22/7.f)};

    /// Octahedron (8 triangles)
    /// Bottom half
    scene_objects.push_back(new Triangle(o_rot*Vector{ 0,-1, 0}+Vector{0,1,0},
                                         o_rot*Vector{-1, 0, 0}+Vector{0,1,0},
                                         o_rot*Vector{ 0, 0, 1}+Vector{0,1,0}));
    scene_objects.back()->set_diffuse_factor(0);
    scene_objects.push_back(new Triangle(o_rot*Vector{ 0,-1, 0}+Vector{0,1,0},
                                         o_rot*Vector{ 0, 0,-1}+Vector{0,1,0},
                                         o_rot*Vector{-1, 0, 0}+Vector{0,1,0}));
    scene_objects.back()->set_diffuse_factor(0);
    scene_objects.push_back(new Triangle(o_rot*Vector{ 0,-1, 0}+Vector{0,1,0},
                                         o_rot*Vector{ 1, 0, 0}+Vector{0,1,0},
                                         o_rot*Vector{ 0, 0,-1}+Vector{0,1,0}));
    scene_objects.back()->set_diffuse_factor(0);
    scene_objects.push_back(new Triangle(o_rot*Vector{ 0,-1, 0}+Vector{0,1,0},
                                         o_rot*Vector{ 0, 0, 1}+Vector{0,1,0},
                                         o_rot*Vector{ 1, 0, 0}+Vector{0,1,0}));
    scene_objects.back()->set_diffuse_factor(0);
    /// Top half
    scene_objects.push_back(new Triangle(o_rot*Vector{ 0, 1, 0}+Vector{0,1,0},
                                         o_rot*Vector{ 0, 0, 1}+Vector{0,1,0},
                                         o_rot*Vector{-1, 0, 0}+Vector{0,1,0}));
    scene_objects.back()->set_diffuse_factor(0);
    scene_objects.push_back(new Triangle(o_rot*Vector{ 0, 1, 0}+Vector{0,1,0},
                                         o_rot*Vector{ 1, 0, 0}+Vector{0,1,0},
                                         o_rot*Vector{ 0, 0, 1}+Vector{0,1,0}));
    scene_objects.back()->set_diffuse_factor(0);
    scene_objects.push_back(new Triangle(o_rot*Vector{ 0, 1, 0}+Vector{0,1,0},
                                         o_rot*Vector{ 0, 0,-1}+Vector{0,1,0},
                                         o_rot*Vector{ 1, 0, 0}+Vector{0,1,0}));
    scene_objects.back()->set_diffuse_factor(0);
    scene_objects.push_back(new Triangle(o_rot*Vector{ 0, 1, 0}+Vector{0,1,0},
                                         o_rot*Vector{-1, 0, 0}+Vector{0,1,0},
                                         o_rot*Vector{ 0, 0,-1}+Vector{0,1,0}));
    scene_objects.back()->set_diffuse_factor(0);


    const int max_hit_bounces{6};
    const int pixel_samples{4};


    /// PPM header 
    /// The magic number "P6" means "color PPM in binary format (= not ASCII)"
    outfile << "P6 512 512 255 ";

    for(int y = 256; y >= -255; --y) {   //For each row
      std::cout << "Progress: " 
                << std::setw(3) << (int)(100*(-y+256+1)/512.f) 
                << "%\r" << std::flush;
      for(int x = -255; x <= 256; ++x) {   //For each pixel in a row

        Vector final_color{0,0,0};

        for (size_t sample = 0; sample < pixel_samples; ++sample) {

          Vector color{0,0,0};

          /// Random sensor shift for depth-of-field
          Vector sensor_shift{uniform_random_01()*0.1f,
                              uniform_random_01()*0.1f,
                              0};

          RotationMatrix camera_rotation{-0.2f*std::sin(frame_id/100.f*(22/7.f)),
                                         frame_id/100.f*(22/7.f),
                                         0.2f*std::sin(frame_id/100.f*(22/7.f))};

          Vector ray_origin{0,1,-4};
          /// Random offset on ray direction for antialiasing
          Vector ray_direction{right*(x-0.5+uniform_random_01()) + 
                                  up*(y-0.5+uniform_random_01()) + 
                               ahead};

          ray_origin = camera_rotation * (ray_origin + sensor_shift);
          ray_direction = !(ray_direction - sensor_shift*(1./4));
          ray_direction = camera_rotation*!ray_direction;

          Vector ray_hit_at, 
                 ray_bounced_direction, 
                 normal,
                 hit_color;
          float distance_to_hit, 
                diffuse_factor_at_hit, 
                specular_factor_at_hit, 
                reflectivity_at_hit, 
                ray_energy_left=1;

          for (int bounce = 0; bounce <= max_hit_bounces; ++bounce) {
            /// Compute object intersections
            bool an_object_was_hit{false};
            bool sky_hit{false};
            float min_hit_distance{std::numeric_limits<float>::max()};
            Object* closest_object_ptr{nullptr};
            for (const auto& object : scene_objects) {
              if (object->is_hit_by_ray(ray_origin, ray_direction,
                                        ray_hit_at, ray_bounced_direction,
                                        normal,
                                        distance_to_hit,
                                        hit_color,
                                        diffuse_factor_at_hit,
                                        specular_factor_at_hit,
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
                                                normal,
                                                distance_to_hit,
                                                hit_color,
                                                diffuse_factor_at_hit,
                                                specular_factor_at_hit,
                                                reflectivity_at_hit);
              ray_origin = ray_hit_at;
              ray_direction = ray_bounced_direction;
            } else {
              if (ray_direction.y < 0) {
                hit_color = get_ground_color(ray_origin, ray_direction, ray_hit_at);
                //normal = {0,1,0};
                //ray_bounced_direction = !(ray_direction + !normal*(-ray_direction%!normal)*2);
                ray_bounced_direction = !(Vector{uniform_random_01(),
                                                 0.5f+uniform_random_01(),
                                                 uniform_random_01()});
                normal = !(ray_bounced_direction - ray_direction);
                ray_origin = ray_hit_at;
                ray_direction = ray_bounced_direction;
                reflectivity_at_hit = 0;
                diffuse_factor_at_hit = 1;
                specular_factor_at_hit = 0;
              } else {
                hit_color = get_sky_color(ray_direction);
                reflectivity_at_hit = 0;
                sky_hit = true;
              }
            }


            /// Compute lighting at hit point
            Vector light_at{uniform_random_01()*10,
                            100,
                            uniform_random_01()*10};
            bool point_is_directly_lit{true};
            for (const auto& object : scene_objects) {
              /// Dummy variables
              Vector a,b,c,f;
              float d,e,di,sp;
              if (object->is_hit_by_ray(ray_hit_at, 
                                        !(light_at-ray_hit_at), 
                                        a, b, f, d, c, di, sp, e))
              {
                point_is_directly_lit = false;
                break; 
              }
            }
            if (not sky_hit) {
              const float ambient_light = 0.3;
              if (point_is_directly_lit) {
                const float diffuse_light = std::max(0.f, normal%!(light_at-ray_hit_at));
                const float specular_light = std::max(0.f, !(light_at-ray_hit_at)%ray_direction);
                hit_color = hit_color*(ambient_light+diffuse_light)*diffuse_factor_at_hit + 
                            Vector{255,255,255}*2*std::pow(specular_light,99)*specular_factor_at_hit;
              } else {
                hit_color = hit_color*ambient_light*diffuse_factor_at_hit;
              }
            }


            color = color + (hit_color*(ray_energy_left));
            ray_energy_left *= reflectivity_at_hit;
            if (ray_energy_left <= 0)
              break;
          }

          final_color = final_color + color;

        }

        final_color = final_color * (1.0/pixel_samples);

        /// Write this pixel's RGB color triple (each a single byte)
        outfile << (unsigned char)(std::max(0.f, std::min(final_color.r, 255.f)))
                << (unsigned char)(std::max(0.f, std::min(final_color.g, 255.f)))
                << (unsigned char)(std::max(0.f, std::min(final_color.b, 255.f)));

      }
    }

    /// Image done
    outfile.close();
  }

  /// Bye!
  return EXIT_SUCCESS;
}


