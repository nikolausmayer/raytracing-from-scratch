
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <vector>


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



class Object
{
  public:
  Object()
    : color(255, 255, 255),
      reflectivity(1)
  { }

  virtual bool is_hit_by_ray(const Vector& incoming_ray_origin,
                             const Vector& incoming_ray_direction,
														 Vector& outgoing_ray_origin,
														 Vector& outgoing_ray_direction,
                             float& hit_distance,
                             Vector& hit_color,
                             float& object_reflectivity) const = 0;

  void set_color(const Vector& v)
  {
    color = v;
  }

  void set_reflectivity(float v)
  {
    reflectivity = v;
  }


  Vector color;
  float reflectivity;
};



class Triangle : public Object
{
public:
  Triangle(const Vector& p0,
           const Vector& p1,
           const Vector& p2)
    : p0(p0), p1(p1), p2(p2)
  {
    u = p1 - p0;
    v = p2 - p0;
    normal = v^u;
  }

  Vector p0, p1, p2, u, v, normal;

  bool is_hit_by_ray(const Vector& incoming_ray_origin,
                     const Vector& incoming_ray_direction,
										 Vector& outgoing_ray_origin,
										 Vector& outgoing_ray_direction,
                     float& hit_distance,
                     Vector& hit_color,
                     float& object_reflectivity) const
  {
    if (normal % incoming_ray_direction >= 0)
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

		if (u_factor < 0 or u_factor > 1 or
				v_factor < 0 or v_factor > 1 or
				u_factor+v_factor > 1 or
				ray_factor < 0)
			return false;
    
		hit_distance = (incoming_ray_direction*ray_factor).length();

    if (hit_distance < 1e-3)
      return false;

		outgoing_ray_origin = p0 + u*u_factor + v*v_factor;
		outgoing_ray_direction = !(incoming_ray_direction + 
														  !normal*(!normal%-incoming_ray_direction)*2);

		hit_color = color;
    object_reflectivity = reflectivity;

		return true;
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



  std::vector<Object*> scene_objects;

  scene_objects.push_back(new Triangle({-2,0,-1},
                                       {2,0,-1},
                                       {0,3,-1}));
  scene_objects.back()->set_color({0,0,255});
  scene_objects.back()->set_reflectivity(0.8);
  scene_objects.push_back(new Triangle({2,0,-5},
                                       {-2,0,-5},
                                       {0,3,-5}));
  scene_objects.back()->set_color({0,255,0});
  scene_objects.back()->set_reflectivity(0.8);


  std::ofstream outfile("img.ppm", std::ios::binary);

  /// PPM Header
  outfile << "P6 512 512 255 ";

  for (int y = 256; y >= -255; --y) {
    std::cout << std::setw(3) << 100*(256-y+1)/512 << "%\r" << std::flush;
    for (int x = -255; x <= 256; ++x) {

      Vector final_color{0, 0, 0};

      Vector ray_origin{0, 1, -4};
      Vector ray_direction = !Vector{X*(x-0.5f) + 
                                     Y*(y-0.5f) +
                                     Z};


      Vector ray_hit_at;
      Vector ray_bounced_direction;
      float distance_to_hit;
      float reflectivity_at_hit;
      float ray_energy_left = 1.f;

      for (int bounce = 0; bounce <= 100; ++bounce) {
        Vector color{0, 0, 0};

        bool an_object_was_hit{false};
        float min_hit_distance{std::numeric_limits<float>::max()};
        Object* closest_object_ptr{nullptr};

        for (const auto& object : scene_objects) {
          if (object->is_hit_by_ray(ray_origin,
                                    ray_direction,
                                    ray_hit_at,
                                    ray_bounced_direction,
                                    distance_to_hit,
                                    color,
                                    reflectivity_at_hit)) {
            an_object_was_hit = true;
            if (distance_to_hit < min_hit_distance) {
              min_hit_distance = distance_to_hit;
              closest_object_ptr = object;
            }
          }
        }

        if (an_object_was_hit and closest_object_ptr) {
          closest_object_ptr->is_hit_by_ray(ray_origin,
                                            ray_direction,
                                            ray_hit_at,
                                            ray_bounced_direction,
                                            distance_to_hit,
                                            color,
                                            reflectivity_at_hit);
          ray_origin = ray_hit_at;
          ray_direction = ray_bounced_direction;
        } else {
          if (ray_direction.y < 0) {
            color = get_ground_color(ray_origin, ray_direction);
            reflectivity_at_hit = 0.f;
          } else {
            color = get_sky_color(ray_direction);
            reflectivity_at_hit = 0.f;
          }
        }


        final_color = final_color + (color*(ray_energy_left*(1-reflectivity_at_hit)));
        ray_energy_left *= reflectivity_at_hit;
        if (ray_energy_left <= 0)
          break;
      }  /// <-- bounce loop end


      outfile 
      << static_cast<unsigned char>(std::max(0.f, std::min(255.f, final_color.x)))
      << static_cast<unsigned char>(std::max(0.f, std::min(255.f, final_color.y)))
      << static_cast<unsigned char>(std::max(0.f, std::min(255.f, final_color.z)));
    }
  }

  ///
  return EXIT_SUCCESS;
}

