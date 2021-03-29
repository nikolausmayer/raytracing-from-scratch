
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <vector>


float random_offset()
{
  // Random value uniform in [-0.5, 0.5]
  return static_cast<float>(rand()) / RAND_MAX - 0.5f;
  //return 0.f;
}


class Vector {
public:
  /// Constructor
  constexpr Vector(float x=0.f, float y=0.f, float z=0.f)
    : x(x), y(y), z(z)
  { }

  float x;
  float y;
  float z;

  /// Addition
  constexpr Vector operator+(const Vector& rhs) const
  {
    return Vector{x+rhs.x,
                  y+rhs.y,
                  z+rhs.z};
  }

  /// Subtraction
  constexpr Vector operator-(const Vector& rhs) const
  {
    return *this + rhs*-1;
  }

  /// Negation
  constexpr Vector operator-() const
  {
    return *this * -1;
  }

  /// Linear scaling
  constexpr Vector operator*(float r) const
  {
    return Vector{x*r, y*r, z*r};
  }

  /// Dot product
  constexpr float operator%(const Vector& rhs) const
  {
    return x*rhs.x + y*rhs.y + z*rhs.z;
  }

  /// Cross product
  constexpr Vector operator^(const Vector& rhs) const
  {
    return Vector{y*rhs.z - z*rhs.y,
                  z*rhs.x - x*rhs.z,
                  x*rhs.y - y*rhs.x};
  }

  /// Vector length
  constexpr float length() const
  {
    return std::sqrt(x*x + y*y + z*z);
  }

  /// Normalization to unit length
  constexpr Vector operator!() const
  {
    return (*this)*(1.f/length());
  }
};



class RotationMatrix
{
public:
  RotationMatrix(float x, float y, float z)
  {
    *this = RotationMatrix(1.0f, 0.0f, 0.0f,
                           0.0f, std::cos(x), -std::sin(x),
                           0.0f, std::sin(x),  std::cos(x)) *
            RotationMatrix(std::cos(y), 0.0f, std::sin(y),
                           0.0f, 1.0f, 0.0f,
                           -std::sin(y), 0.0f, std::cos(y)) *
            RotationMatrix(std::cos(z), -std::sin(z), 0.0f,
                           std::sin(z),  std::cos(z), 0.0f,
                           0.0f, 0.0f, 1.0f);
  }

  RotationMatrix(float a, float b, float c,
                 float d, float e, float f,
                 float g, float h, float i)
  {
    data[0][0] = a; data[0][1] = b; data[0][2] = c;
    data[1][0] = d; data[1][1] = e; data[1][2] = f;
    data[2][0] = g; data[2][1] = h; data[2][2] = i;
  }

  RotationMatrix operator*(const RotationMatrix& R) const
  {
    RotationMatrix result(0,0,0, 0,0,0, 0,0,0);
    for (size_t y = 0; y < 3; ++y) {
      for (size_t x = 0; x < 3; ++x) {
        result.data[y][x] = data[y][0] * R.data[0][x] +
                            data[y][1] * R.data[1][x] + 
                            data[y][2] * R.data[2][x];
      }
    }
    return result;
  }

  Vector operator*(const Vector& V) const
  {
    return Vector{data[0][0] * V.x + data[0][1] * V.y + data[0][2] * V.z,
                  data[1][0] * V.x + data[1][1] * V.y + data[1][2] * V.z,
                  data[2][0] * V.x + data[2][1] * V.y + data[2][2] * V.z};
  }

  float data[3][3];
};


Vector sample_texture(int u, int v)
{
  //if (u < 0 or v < 0 or u >= 1 or v >= 1)
  //  return {0,0,0};

  static unsigned char* texture_data{nullptr};
  const int tex_w{600};
  const int tex_h{400};
  if (not texture_data) {
    std::ifstream texture("texture-sd.ppm", std::ios::binary);
    texture_data = new unsigned char[tex_w * tex_h * 3];
    texture.read(reinterpret_cast<char*>(texture_data), 15);
    texture.read(reinterpret_cast<char*>(texture_data), tex_w * tex_h * 3);
  }

  const size_t color_start_idx = (v * tex_w + u) * 3;
  return Vector{static_cast<float>(texture_data[color_start_idx]),
                static_cast<float>(texture_data[color_start_idx+1]),
                static_cast<float>(texture_data[color_start_idx+2])};
}


class Object
{
  public:
  Object()
    : color(255, 255, 255),
      reflectivity(1),
      hardness(1),
      diffuse_factor(1),
      specular_factor(1),
      roughness(0)
  { }

  virtual bool is_hit_by_ray(const Vector& incoming_ray_origin,
                             const Vector& incoming_ray_direction,
														 Vector& outgoing_ray_origin,
														 Vector& outgoing_ray_direction,
                             float& hit_distance,
                             Vector& hit_color,
                             float& object_reflectivity,
                             float& object_hardness,
                             float& object_diffuse_factor,
                             float& object_specular_factor,
                             Vector& object_normal) const = 0;

  void set_color(const Vector& v)
  {
    color = v;
  }

  void set_reflectivity(float v)
  {
    reflectivity = v;
  }

  void set_hardness(float v)
  {
    hardness = v;
  }

  void set_diffuse_factor(float v)
  {
    diffuse_factor = v;
  }

  void set_specular_factor(float v)
  {
    specular_factor = v;
  }

  void set_roughness(float v)
  {
    roughness = v;
  }


  Vector color;
  float reflectivity;
  float hardness;
  float diffuse_factor;
  float specular_factor;
  float roughness;
};



class Sphere : public Object
{
public:
  Sphere(const Vector& center,
         float radius)
    : center(center),
      radius(radius)
  { }

  Vector center;
  float radius;


  bool is_hit_by_ray(const Vector& incoming_ray_origin,
                     const Vector& incoming_ray_direction,
										 Vector& outgoing_ray_origin,
										 Vector& outgoing_ray_direction,
                     float& hit_distance,
                     Vector& hit_color,
                     float& object_reflectivity,
                     float& object_hardness,
                     float& object_diffuse_factor,
                     float& object_specular_factor,
                     Vector& object_normal) const
  {
    const Vector p = center - incoming_ray_origin;
    const float threshold = std::sqrt(p%p - radius*radius);
    const float b = p % incoming_ray_direction;

    if (b > threshold) {
      /// HIT
      const float s = std::sqrt(p % p - b * b);
      const float t = std::sqrt(radius * radius - s * s);
      hit_distance = b - t;

      if (hit_distance < 1e-3)
        return false;

      outgoing_ray_origin = incoming_ray_origin + incoming_ray_direction*hit_distance;
      const Vector normal = !(-p + incoming_ray_direction * hit_distance);
      outgoing_ray_direction = !(incoming_ray_direction + 
                                !normal*(!normal%-incoming_ray_direction)*2);

      /// Add randomness (would be the same for triangles)
      outgoing_ray_direction = !(outgoing_ray_direction +
                                 !Vector{random_offset(),
                                         random_offset(),
                                         random_offset()}*roughness);
      if (normal % outgoing_ray_direction <= 0) {
        outgoing_ray_direction = !(outgoing_ray_direction + 
                                  !normal*(!normal%-outgoing_ray_direction)*2);
      }
      object_normal = !(outgoing_ray_direction - incoming_ray_direction);
 

      hit_color = color;
      object_reflectivity = reflectivity;

      object_hardness = hardness;
      object_diffuse_factor = diffuse_factor;
      object_specular_factor = specular_factor;

      //object_normal = normal;

      return true;
    } else {
      /// NO HIT
      return false;
    }
  }
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
                     float& object_reflectivity,
                     float& object_hardness,
                     float& object_diffuse_factor,
                     float& object_specular_factor,
                     Vector& object_normal) const
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

    /// Add randomness (would be the same for triangles)
    outgoing_ray_direction = !(outgoing_ray_direction +
                               !Vector{random_offset(),
                                       random_offset(),
                                       random_offset()}*roughness);
    if (normal % outgoing_ray_direction <= 0) {
      outgoing_ray_direction = !(outgoing_ray_direction + 
                                !normal*(!normal%-outgoing_ray_direction)*2);
    }
    object_normal = !(outgoing_ray_direction - incoming_ray_direction);

		//hit_color = color;
    {
      const Vector p0p{incoming_ray_origin + !(p0 - incoming_ray_origin)};
      const float& pox{p0p.x};
      const float& poy{p0p.y};
      const float& poz{p0p.z};
      const Vector up{(incoming_ray_origin + !(p1 - incoming_ray_origin)) - p0p};
      const float& ux{up.x};
      const float& uy{up.y};
      const float& uz{up.z};
      const Vector vp{(incoming_ray_origin + !(p2 - incoming_ray_origin)) - p0p};
      const float& vx{vp.x};
      const float& vy{vp.y};
      const float& vz{vp.z};
      const float& rx{incoming_ray_direction.x};
      const float& ry{incoming_ray_direction.y};
      const float& rz{incoming_ray_direction.z};
      const float& ox{incoming_ray_origin.x};
      const float& oy{incoming_ray_origin.y};
      const float& oz{incoming_ray_origin.z};
      const float u_factor = (-(ox - pox)*(ry*vz - rz*vy) + (oy - poy)*(rx*vz - rz*vx) - (oz - poz)*(rx*vy - ry*vx))/(rx*uy*vz - rx*uz*vy - ry*ux*vz + ry*uz*vx + rz*ux*vy - rz*uy*vx);
      const float v_factor = ((ox - pox)*(ry*uz - rz*uy) - (oy - poy)*(rx*uz - rz*ux) + (oz - poz)*(rx*uy - ry*ux))/(rx*uy*vz - rx*uz*vy - ry*ux*vz + ry*uz*vx + rz*ux*vy - rz*uy*vx);

      hit_color = sample_texture(u_factor*100, v_factor*50);
    }
    object_reflectivity = reflectivity;

    object_hardness = hardness;
    object_diffuse_factor = diffuse_factor;
    object_specular_factor = specular_factor;

    //object_normal = !normal;

		return true;
  }
};



Vector get_ground_color(const Vector& ray_origin,
                        const Vector& ray_direction,
                        Vector& ray_hit_at)
{
  //return {255, 0, 0};
  const float distance = -ray_origin.y / ray_direction.y;
  const float x = ray_origin.x + ray_direction.x * distance;
  const float z = ray_origin.z + ray_direction.z * distance;

  ray_hit_at = {x, 0, z};

  const int tex_u = std::abs((int)((x*50)+1000)) % 600;
  const int tex_v = std::abs((int)((z*50)+1100)) % 400;
  return sample_texture(tex_u, tex_v);

  //if ((int)std::abs(std::floor(x)) % 2 ==
  //    (int)std::abs(std::floor(z)) % 2 )
  //{
  //  return {255, 0, 0};
  //} else {
  //  return {255, 255, 255};
  //}
}

Vector get_sky_color(const Vector& ray_direction)
{
  //return {0, 0, 255};
  return Vector{0.7, 0.6, 1.0}*255 * std::pow(1-ray_direction.y, 2);
}


int main() {
  constexpr auto pixelization_factor{2};

  /// LEFT-hand coordinate system
  /// X = right
  const Vector X{0.002, 0, 0};
  /// Y = up
  const Vector Y{0, 0.002, 0};
  /// Z = forwards
  const Vector Z{0, 0, 1};



  for (int frame = 0; frame < 1; ++frame) {

    std::vector<Object*> scene_objects;

    scene_objects.push_back(new Triangle{{1.5, 2, 1},
                                         {-1.5, 2, 1},
                                         {1.5, 0, 0}});
    scene_objects.back()->set_roughness(0);
    scene_objects.back()->set_reflectivity(0);
    scene_objects.back()->set_specular_factor(0);
    scene_objects.back()->set_color({255,0,0});
    scene_objects.push_back(new Triangle{{-1.5, 0, 0},
                                         {1.5, 0, 0},
                                         {-1.5, 2, 1}});
    scene_objects.back()->set_roughness(0);
    scene_objects.back()->set_reflectivity(0);
    scene_objects.back()->set_specular_factor(0);
    scene_objects.back()->set_color({255,0,0});

    //scene_objects.push_back(new Sphere({1, 2, 0}, 0.5));
    //scene_objects.back()->set_color({0, 0, 0});
    //scene_objects.back()->set_reflectivity(0.95);
    //scene_objects.back()->set_diffuse_factor(0);
    //scene_objects.back()->set_roughness(0.75);
    //scene_objects.push_back(new Sphere({-1.25, 0.8, 0}, 0.25));
    //scene_objects.back()->set_color({255, 165, 0});
    //scene_objects.back()->set_reflectivity(0.05);
    //scene_objects.back()->set_diffuse_factor(0.9);
    //scene_objects.back()->set_specular_factor(1);
    //scene_objects.back()->set_hardness(99);

    ///// The octahedron has a separate rotation
    //RotationMatrix o_rot{0.5f*frame/100.f*(22/7.f),
    //                     0.5f*frame/100.f*(22/7.f),
    //                     0.5f*frame/100.f*(22/7.f)};

    ///// Octahedron (8 triangles)
    ///// Bottom half
    //scene_objects.push_back(new Triangle(o_rot*Vector{ 0,-1, 0}+Vector{0,1,0},
    //                                     o_rot*Vector{-1, 0, 0}+Vector{0,1,0},
    //                                     o_rot*Vector{ 0, 0, 1}+Vector{0,1,0}));
    //scene_objects.back()->set_diffuse_factor(0);
    //scene_objects.push_back(new Triangle(o_rot*Vector{ 0,-1, 0}+Vector{0,1,0},
    //                                     o_rot*Vector{ 0, 0,-1}+Vector{0,1,0},
    //                                     o_rot*Vector{-1, 0, 0}+Vector{0,1,0}));
    //scene_objects.back()->set_diffuse_factor(0);
    //scene_objects.push_back(new Triangle(o_rot*Vector{ 0,-1, 0}+Vector{0,1,0},
    //                                     o_rot*Vector{ 1, 0, 0}+Vector{0,1,0},
    //                                     o_rot*Vector{ 0, 0,-1}+Vector{0,1,0}));
    //scene_objects.back()->set_diffuse_factor(0);
    //scene_objects.push_back(new Triangle(o_rot*Vector{ 0,-1, 0}+Vector{0,1,0},
    //                                     o_rot*Vector{ 0, 0, 1}+Vector{0,1,0},
    //                                     o_rot*Vector{ 1, 0, 0}+Vector{0,1,0}));
    //scene_objects.back()->set_diffuse_factor(0);
    ///// Top half
    //scene_objects.push_back(new Triangle(o_rot*Vector{ 0, 1, 0}+Vector{0,1,0},
    //                                     o_rot*Vector{ 0, 0, 1}+Vector{0,1,0},
    //                                     o_rot*Vector{-1, 0, 0}+Vector{0,1,0}));
    //scene_objects.back()->set_diffuse_factor(0);
    //scene_objects.push_back(new Triangle(o_rot*Vector{ 0, 1, 0}+Vector{0,1,0},
    //                                     o_rot*Vector{ 1, 0, 0}+Vector{0,1,0},
    //                                     o_rot*Vector{ 0, 0, 1}+Vector{0,1,0}));
    //scene_objects.back()->set_diffuse_factor(0);
    //scene_objects.push_back(new Triangle(o_rot*Vector{ 0, 1, 0}+Vector{0,1,0},
    //                                     o_rot*Vector{ 0, 0,-1}+Vector{0,1,0},
    //                                     o_rot*Vector{ 1, 0, 0}+Vector{0,1,0}));
    //scene_objects.back()->set_diffuse_factor(0);
    //scene_objects.push_back(new Triangle(o_rot*Vector{ 0, 1, 0}+Vector{0,1,0},
    //                                     o_rot*Vector{-1, 0, 0}+Vector{0,1,0},
    //                                     o_rot*Vector{ 0, 0,-1}+Vector{0,1,0}));
    //scene_objects.back()->set_diffuse_factor(0);


    std::ostringstream oss;
    /// "frame_0005.ppm"
    oss << "frame_" << std::setw(4) << std::setfill('0') << frame << ".ppm";
    std::ofstream outfile(oss.str(), std::ios::binary);
    std::cout << oss.str() << std::endl;

    /// PPM Header
    outfile << "P6 512 512 255 ";

    unsigned char cacheline[3 * 512 / pixelization_factor];

    for (int y = 512 / 2 / pixelization_factor; y >= 1 - 512 / 2 / pixelization_factor; --y) {
      auto cachelineindex = 0u;
      //std::cout << std::setw(3) << 100*(256-y+1)/512 << "%\r" << std::flush;
      for (int x = 1 - 512 / 2 / pixelization_factor; x <= 512 / 2 / pixelization_factor; ++x) {

        RotationMatrix camera_rotation{-0.2f*std::sin(frame/100.f*(22/7.f)),
                                        frame/100.f*(22/7.f),
                                        0.2f*std::sin(frame/100.f*(22/7.f))};

        Vector AA_color{0, 0, 0};

        const int AA_samples{1};
        for (int sample = 0; sample < AA_samples; ++sample) {

          Vector color{0, 0, 0};
          Vector final_color{0, 0, 0};

          Vector ray_origin{0, 1, -4};
          Vector ray_direction = !Vector{X*(x*pixelization_factor-0.5f/*+random_offset()*/) + 
                                         Y*(y*pixelization_factor-0.5f/*+random_offset()*/) +
                                         Z};

          /// Depth-of-field
          Vector sensor_shift{0/*+random_offset()*0.1f*/,
                              0/*+random_offset()*0.1f*/,
                              0};
          ray_origin = camera_rotation * (ray_origin + sensor_shift);
          ray_direction = !(ray_direction - sensor_shift*(1./4.));
          ray_direction = camera_rotation * ray_direction;


          Vector ray_hit_at;
          Vector ray_bounced_direction;
          Vector normal;
          float distance_to_hit;
          float reflectivity_at_hit;
          float ray_energy_left = 1.f;
          float hardness_at_hit;
          float diffuse_factor_at_hit;
          float specular_factor_at_hit;

          for (int bounce = 0; bounce < 2; ++bounce) {
            bool an_object_was_hit{false};
            bool sky_was_hit{false};
            float min_hit_distance{std::numeric_limits<float>::max()};
            Object* closest_object_ptr{nullptr};

            for (const auto& object : scene_objects) {
              if (object->is_hit_by_ray(ray_origin,
                                        ray_direction,
                                        ray_hit_at,
                                        ray_bounced_direction,
                                        distance_to_hit,
                                        color,
                                        reflectivity_at_hit,
                                        hardness_at_hit,
                                        diffuse_factor_at_hit,
                                        specular_factor_at_hit,
                                        normal)) {
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
                                                reflectivity_at_hit,
                                                hardness_at_hit,
                                                diffuse_factor_at_hit,
                                                specular_factor_at_hit,
                                                normal);
              ray_origin = ray_hit_at;
              ray_direction = ray_bounced_direction;
            } else {
              if (ray_direction.y < 0) {
                color = get_ground_color(ray_origin, ray_direction, ray_hit_at);
                normal = {0, 1, 0};
                ray_bounced_direction = !(ray_direction+!normal*(-ray_direction%normal)*2);
                ray_origin = ray_hit_at;
                ray_direction = ray_bounced_direction;

                reflectivity_at_hit = 0.f;
                hardness_at_hit = 0;
                diffuse_factor_at_hit = 0.8;
                specular_factor_at_hit = 0;
              } else {
                color = get_sky_color(ray_direction);
                reflectivity_at_hit = 0.f;
                hardness_at_hit = 0;
                specular_factor_at_hit = 0;
                sky_was_hit = true;
              }
            }


            const Vector light_at{  0/*+random_offset()*30*/, 
                                  100,
                                    0/*+random_offset()*30*/};
            const Vector light_color{255,255,255};

            bool point_is_directly_lit{true};
            for (const auto& object : scene_objects) {
              Vector a,b,d,i;
              float c,e,f,g,h;
              if (object->is_hit_by_ray(ray_hit_at,
                                        !(light_at - ray_hit_at), 
                                        a,b,c,d,e,f,g,h,i)) {
                point_is_directly_lit = false;
                break;
              }
            }

            if (not sky_was_hit) {
              const float ambient_light{0.3f};
              if (point_is_directly_lit) {
                const float diffuse_light{std::max(0.f, normal % !(light_at - ray_hit_at))};
                const float specular_factor{std::max(0.f, !(light_at - ray_hit_at) % ray_direction)};
                color = color * ambient_light +
                        color * diffuse_light * diffuse_factor_at_hit +
                        light_color * std::pow(specular_factor, hardness_at_hit) * specular_factor_at_hit;
              } else {
                color = color * ambient_light;
              }
            }


            final_color = final_color + (color*(ray_energy_left*(1-reflectivity_at_hit)));
            ray_energy_left *= reflectivity_at_hit;
            if (ray_energy_left <= 0)
              break;
          }  /// <-- bounce loop end

          AA_color = AA_color + final_color*(1./AA_samples);
            
        }  /// <-- antialiasing loop end


        cacheline[cachelineindex++] = static_cast<unsigned char>(std::max(0.f, std::min(255.f, AA_color.x)));
        cacheline[cachelineindex++] = static_cast<unsigned char>(std::max(0.f, std::min(255.f, AA_color.y)));
        cacheline[cachelineindex++] = static_cast<unsigned char>(std::max(0.f, std::min(255.f, AA_color.z)));
      }

      for (auto i = 0; i < pixelization_factor; ++i) {
        for (auto j = 0; j < 3 * 512 / pixelization_factor; j+=3) {
          for (auto k = 0; k < pixelization_factor; ++k) {
            outfile << cacheline[j];
            outfile << cacheline[j+1];
            outfile << cacheline[j+2];
          }
        }
      }
    }

  }  /// <-- frames loop end

  ///
  return EXIT_SUCCESS;
}

