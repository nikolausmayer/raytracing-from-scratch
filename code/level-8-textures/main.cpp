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
    : roughness(0),
      reflectivity(1),
      color({255,255,255})
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
                             float& object_reflectivity) const = 0;

  void set_roughness(float v)
  {
    roughness = v;
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
                     Vector& outgoing_normal,
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
    //const Vector n = normal;
    const Vector n = !(normal+Vector(uniform_random_01(),
                                     uniform_random_01(),
                                     uniform_random_01())*roughness);

    outgoing_ray_origin = p0+v*u_factor+u*v_factor;
    outgoing_ray_direction = !(incoming_ray_direction - !n*(incoming_ray_direction%!n)*2);
    outgoing_normal = n;
    hit_color = color;
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
                     float& object_reflectivity) const
  {
    const Vector p=incoming_ray_origin-center;
    float b=p%incoming_ray_direction,
          c=p%p-radius*radius,
          q=b*b-c;

    //Does the ray hit the sphere ?
    if (q > 0) {
      //It does, compute the distance camera-sphere
      float s=-b-sqrt(q);

      if (s < 1e-3)
        return false;

      hit_distance=s;
      outgoing_ray_origin = incoming_ray_origin + incoming_ray_direction*hit_distance;
      //const Vector normal = !(p+incoming_ray_direction*hit_distance);
      const Vector normal = !(!(p+incoming_ray_direction*hit_distance) +
                             Vector(uniform_random_01(),
                                    uniform_random_01(),
                                    uniform_random_01())*roughness);
      outgoing_ray_direction = !(incoming_ray_direction - normal*(incoming_ray_direction%normal)*2);
      outgoing_normal = normal;

      hit_color = color;
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

  /// 2 spheres
  scene_objects.push_back(new Sphere(Vector{1,2,0}, .5));
  scene_objects.back()->set_roughness(0.75);
  scene_objects.back()->set_reflectivity(0.95);
  scene_objects.back()->set_color({0,0,0});
  scene_objects.push_back(new Sphere(Vector{-1.25,.8,0}, .25));
  scene_objects.back()->set_reflectivity(0.05);
  scene_objects.back()->set_color({255,0,0});

  /// Octahedron (8 triangles)
  /// Bottom half
  scene_objects.push_back(new Triangle(Vector{0,0,0},
                                       Vector{0,1,1},
                                       Vector{-1,1,0}));
  scene_objects.push_back(new Triangle(Vector{0,0,0},
                                       Vector{-1,1,0},
                                       Vector{0,1,-1}));
  scene_objects.push_back(new Triangle(Vector{0,0,0},
                                       Vector{0,1,-1},
                                       Vector{1,1,0}));
  scene_objects.push_back(new Triangle(Vector{0,0,0},
                                       Vector{1,1,0},
                                       Vector{0,1,1}));
  /// Top half
  scene_objects.push_back(new Triangle(Vector{0,2,0},
                                       Vector{0,1,1},
                                       Vector{-1,1,0}));
  scene_objects.push_back(new Triangle(Vector{0,2,0},
                                       Vector{-1,1,0},
                                       Vector{0,1,-1}));
  scene_objects.push_back(new Triangle(Vector{0,2,0},
                                       Vector{0,1,-1},
                                       Vector{1,1,0}));
  scene_objects.push_back(new Triangle(Vector{0,2,0},
                                       Vector{1,1,0},
                                       Vector{0,1,1}));

  const int max_hit_bounces{6};
  const int pixel_samples{64};


  /// PPM header 
  /// The magic number "P6" means "color PPM in binary format (= not ASCII)"
  outfile << "P6 512 512 255 ";

  for(int y = 256; y >= -255; --y) {   //For each row

    std::cout << "Progress: " << std::setw(3) << (int)(100*(-y+256+1)/512.f) << "%\r" << std::flush;

    for(int x = -255; x <= 256; ++x) {   //For each pixel in a row

      Vector final_color{0,0,0};

      for (size_t sample = 0; sample < pixel_samples; ++sample) {

        Vector color{0,0,0};

        /// Random sensor shift for depth-of-field
        Vector sensor_shift{uniform_random_01()*0.1f,
                            uniform_random_01()*0.1f,
                            0};

        Vector ray_origin{0,1,-4};
        /// Random offset on ray direction for antialiasing
        Vector ray_direction{right*(x-0.5+uniform_random_01()) + 
                             down*(y-0.5+uniform_random_01()) + 
                             ahead};

        ray_origin = ray_origin + sensor_shift;
        ray_direction = !(ray_direction - sensor_shift*(1./4));

        Vector ray_hit_at, 
               ray_bounced_direction, 
               normal,
               hit_color;
        float distance_to_hit, 
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
                                              reflectivity_at_hit);
            ray_origin = ray_hit_at;
            ray_direction = ray_bounced_direction;
          } else {
            if (ray_direction.y < 0) {
              hit_color = get_ground_color(ray_origin, ray_direction, ray_hit_at);
              normal = {0,0.1,0};
              reflectivity_at_hit = 0;
            } else {
              hit_color = get_sky_color(ray_direction);
              reflectivity_at_hit = 0;
              sky_hit = true;
            }
          }


          /// Compute lighting at hit point
          /// Random offset to light position for soft shadows and broader
          /// highlights
          Vector light_at{1+uniform_random_01()*10,
                          100+uniform_random_01()*10,
                          0+uniform_random_01()*10};
          bool point_is_directly_lit{true};
          for (const auto& object : scene_objects) {
            Vector a,b,c,f;
            float d,e;
            if (object->is_hit_by_ray(ray_hit_at, 
                                      !(light_at-ray_hit_at), 
                                      a, b, f, d, c, e))
            {
              point_is_directly_lit = false;
              break; 
            }
          }
          if (not sky_hit) {
            if (point_is_directly_lit) {
              const float specular_factor = normal%!(light_at-ray_hit_at);
              const float highlight = std::max(0.f,
                                      std::min(!(light_at-ray_hit_at)%ray_direction,
                                               1.f));
              hit_color = hit_color 
                          + Vector{255,255,255} * (0.5-0.5*reflectivity_at_hit) * specular_factor
                          + Vector{255,255,255} * 10 * std::pow(highlight,99);
            } else {
              hit_color = hit_color*0.3;
            }
          }


          color = color + (hit_color*(ray_energy_left*(1-reflectivity_at_hit)));
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

  std::cout << "Progress: Done!" << std::endl;

  /// Image done
  outfile.close();
}


