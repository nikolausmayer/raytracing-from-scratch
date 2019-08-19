
#include <atomic>
#include <chrono>
#include <cmath>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <random>
#include <sstream>
#include <vector>
#include <thread>


float random_offset()
{
  // Random value uniform in [-0.5, 0.5]
  static thread_local unsigned int seed(time(NULL));
  return static_cast<float>(rand_r(&seed)) / RAND_MAX - 0.5f;
}


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



class Ray
{
public:
  Ray(const Vector& origin, 
      const Vector& direction,
      unsigned char* __restrict__ memory_target=nullptr)
    : origin(origin),
      direction(direction),
      hit_at(0,0,0),
      color(0,0,0),
      energy(1),
      depth(0),
      memory_target(memory_target),
      parent(nullptr),
      is_finalized{false},
      refractive_index{1.f}
  { }

  Ray(const Ray& other)
    : origin(other.origin),
      direction(other.direction),
      hit_at(other.hit_at),
      color(other.color),
      energy(other.energy),
      depth(other.depth),
      memory_target(other.memory_target),
      children(other.children),
      parent(other.parent),
      is_finalized{other.is_finalized}
  { }

  Ray& operator=(const Ray& other)
  {
    origin = other.origin;
    direction = other.direction;
    hit_at = other.hit_at;
    color = other.color;
    energy = other.energy;
    depth = other.depth;
    memory_target = other.memory_target;
    children = other.children;
    parent = other.parent;
    is_finalized = other.is_finalized;

    return *this;
  }

  ~Ray()
  {
    for (auto& child : children)
      if (child)
        delete child;
    children.clear();
  }

  bool try_finalize()
  {
    for (const auto& child : children)
      if (not child->is_finalized)
        return false;

    for (const auto& child : children)
      color = color + child->color*child->energy*energy;

    is_finalized = true;

    if (parent)
      parent->try_finalize();

    return true;
  }

  Vector origin;
  Vector direction;

  Vector hit_at;
  Vector color;
  float hit_distance;
  float object_reflectivity;
  float object_hardness;
  float object_diffuse_factor;
  float object_specular_factor;
  Vector object_normal;

  float energy;
  unsigned int depth;
  unsigned char* memory_target;
  std::vector<Ray*> children;
  Ray* parent;
  bool is_finalized;

  float refractive_index;
};


std::deque<Ray const*> ray_tasks_for_deletion;
std::mutex ray_tasks_for_deletion__mutex;

std::deque<Ray*> ray_tasks_in_flight;
std::mutex ray_tasks_in_flight__mutex;
bool all_pushed{false};
std::atomic<int> ray_counter;

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

  RotationMatrix()
  { }

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



class Object
{
  public:
  Object()
    : color(255, 255, 255),
      reflectivity(1),
      hardness(1),
      diffuse_factor(1),
      specular_factor(1),
      roughness(0),
      refractive_index(1.f)
  { }

  virtual ~Object()
  { }

  virtual bool is_hit_by_ray(Ray* ray) const = 0;

  virtual void trace_object(Ray* ray) const = 0;

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

  void set_refractive_index(float v)
  {
    refractive_index = v;
  }


  Vector color;
  float reflectivity;
  float hardness;
  float diffuse_factor;
  float specular_factor;
  float roughness;
  float refractive_index;
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


  bool is_hit_by_ray(Ray* ray) const
  {
    /// TODO inside->outside on object transmission
    const Vector p = center - ray->origin;
    //const float threshold = std::sqrt(p%p - radius*radius);
    const float threshold_squared = p%p - radius*radius;
    const float b = p % ray->direction;

    if (b*b > threshold_squared) {
      /// HIT
      //const float s = std::sqrt(p % p - b * b);
      const float s_squared = p % p - b * b;
      const float t = std::sqrt(radius * radius - s_squared);
      ray->hit_distance = b - t;

      if (ray->hit_distance < 1e-3)
        return false;

      return true;
    } else {
      /// NO HIT
      return false;
    }
  }

  void trace_object(Ray* ray) const
  {
    const Vector p = center - ray->origin;
    const float b = p % ray->direction;

    /// HIT
    const float s = std::sqrt(p % p - b * b);
    const float t = std::sqrt(radius * radius - s * s);
    ray->hit_distance = b - t;

    const Vector outgoing_ray_origin{ray->origin + 
                                     ray->direction*ray->hit_distance};
    const Vector normal = !(-p + ray->direction * ray->hit_distance);
    Vector outgoing_ray_direction{!(ray->direction + 
                                  !normal*(!normal%-ray->direction)*2)};

    /// Add randomness (would be the same for triangles)
    outgoing_ray_direction = !(outgoing_ray_direction +
                               !Vector{random_offset(),
                                       random_offset(),
                                       random_offset()}*roughness);
    if (normal % outgoing_ray_direction <= 0) {
      outgoing_ray_direction = !(outgoing_ray_direction + 
                                !normal*(!normal%-outgoing_ray_direction)*2);
    }


    ray->origin = outgoing_ray_origin;
    ray->hit_at = outgoing_ray_origin;
    ray->object_normal = !(outgoing_ray_direction - ray->direction);
    ray->direction = outgoing_ray_direction;

    ray->children.emplace_back(new Ray{outgoing_ray_origin,
                                       outgoing_ray_direction});
    ray->children.back()->energy = reflectivity;
    ray->children.back()->parent = ray;
    ray->children.back()->depth  = ray->depth + 1;
    ray->children.back()->refractive_index = refractive_index;


    ray->color = color;
    ray->object_reflectivity = reflectivity;

    ray->object_hardness = hardness;
    ray->object_diffuse_factor = diffuse_factor;
    ray->object_specular_factor = specular_factor;
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

  bool is_hit_by_ray(Ray* ray) const
  {
    /// TODO inside->outside on object transmission
    if (normal % ray->direction >= 0)
      return false;

    /// Moeller-Trumbore algorithm 
    /// https://www.scratchapixel.com/lessons/3d-basic-rendering/ray-tracing-rendering-a-triangle
    {
      const Vector pvec{ray->direction^v};
      const float det{u%pvec};
      if (det > -1e-4) return false;
      //if (det > 1e-4) return false;
      //if (std::abs(det) < 1e-4) return false;
      const float idet{1/det};
      const Vector tvec{ray->origin - p0};
      const float u_factor{tvec%pvec * idet};
      if (u_factor < 0 or u_factor > 1) return false;
      const Vector qvec{tvec^u};
      const float v_factor{ray->direction%qvec * idet};
      if (v_factor < 0 or u_factor+v_factor > 1) return false;
      /// HIT
      ray->hit_distance = v%qvec * idet;
      if (ray->hit_distance < 1e-3) return false;
      return true;
    }
  }

  void trace_object(Ray* ray) const
  {
    const Vector pvec{ray->direction^v};
    const float det{u%pvec};
    const float idet{1/det};
    const Vector tvec{ray->origin - p0};
    const Vector qvec{tvec^u};
    /// HIT
    ray->hit_distance = v%qvec * idet;


    /// Refraction
    /// cos(incident angle)
    const float in_angle = ray->direction%!normal;
    /// refractive-indices-ratio
    const float rir{ray->refractive_index / refractive_index};
    const Vector refracted_direction{
        ray->direction * rir -
        !normal*(rir * in_angle + 
                 std::sqrt(1 - (rir*rir * (1 - in_angle*in_angle))))
    };


    //const Vector outgoing_ray_origin{p0 + u*u_factor + v*v_factor};
    const Vector outgoing_ray_origin{ray->origin + ray->direction*ray->hit_distance};
		Vector outgoing_ray_direction{!(ray->direction + 
														      !normal*(!normal%-ray->direction)*2)};

    /// Add randomness (would be the same for triangles)
    outgoing_ray_direction = !(outgoing_ray_direction +
                               !Vector{random_offset(),
                                       random_offset(),
                                       random_offset()}*roughness);
    if (normal % outgoing_ray_direction <= 0) {
      outgoing_ray_direction = !(outgoing_ray_direction + 
                                !normal*(!normal%-outgoing_ray_direction)*2);
    }
    ray->origin = outgoing_ray_origin;
    ray->direction = outgoing_ray_direction;

    ray->children.emplace_back(new Ray{outgoing_ray_origin,
                                       outgoing_ray_direction});
    ray->children.back()->energy = reflectivity;
    ray->children.back()->parent = ray;
    ray->children.back()->depth  = ray->depth + 1;
    ray->children.back()->refractive_index = refractive_index;

    ray->object_normal = !(outgoing_ray_direction - ray->direction);

		ray->color = color;
    ray->object_reflectivity = reflectivity;
    ray->object_hardness = hardness;
    ray->object_diffuse_factor = diffuse_factor;
    ray->object_specular_factor = specular_factor;
  }
};






void get_ground_color(Ray* ray)
{
  const float distance = -ray->origin.y / ray->direction.y;
  const float x = ray->origin.x + ray->direction.x * distance;
  const float z = ray->origin.z + ray->direction.z * distance;

  static unsigned char* texture_data{nullptr};
  const int tex_w{600};
  const int tex_h{400};
  /// TODO make texture read thread-safe
  if (not texture_data) {
    std::ifstream texture("texture.ppm", std::ios::binary);
    texture_data = new unsigned char[tex_w * tex_h * 3];
    texture.read(reinterpret_cast<char*>(texture_data), 15);
    texture.read(reinterpret_cast<char*>(texture_data), tex_w * tex_h * 3);
  }

  const int tex_u = std::abs((int)((x*100)+1000)) % tex_w;
  const int tex_v = std::abs((int)((z*100)+1100)) % tex_h;
  const size_t color_start_idx = (tex_v * tex_w + tex_u) * 3;
  ray->color = Vector{(float)(texture_data[color_start_idx]),
                      (float)(texture_data[color_start_idx+1]),
                      (float)(texture_data[color_start_idx+2])};

  ray->hit_at = {x, 0, z};
  ray->object_normal = {0, 1, 0};
  ray->object_reflectivity = 0.f;
  ray->object_hardness = 0;
  ray->object_diffuse_factor = 0.8;
  ray->object_specular_factor = 0;
}

void get_sky_color(Ray* ray)
{
  ray->color = Vector{0.7, 0.6, 1.0} * 255 * 
               std::pow(1-ray->direction.y, 2);
  ray->object_reflectivity = 0.f;
  ray->object_hardness = 0;
  ray->object_specular_factor = 0;
}



/// LEFT-hand coordinate system
/// Z = forwards
const Vector Z{0, 0, 1};
/// X = right
const Vector X{0.002, 0, 0};
/// Y = up
const Vector Y{0, 0.002, 0};


const int THREADS{4};
const int AA_samples{64};
const float DoF_jitter{0.f};
const float focus_distance{4.f};


struct World
{
  World() { }

  std::vector<Object*> scene_objects;
  RotationMatrix camera_rotation;
  unsigned char* raw_data;
};



void TraceRayStep(const World& world,
                  Ray* const ray) 
{
  if (ray->depth > 100)
    return;

  bool an_object_was_hit{false};
  bool sky_was_hit{false};
  float min_hit_distance{std::numeric_limits<float>::max()};
  Object const* closest_object_ptr{nullptr};

  for (const auto& object : world.scene_objects) {
    if (object->is_hit_by_ray(ray)) {
      an_object_was_hit = true;
      if (ray->hit_distance < min_hit_distance) {
        min_hit_distance = ray->hit_distance;
        closest_object_ptr = object;
      }
    }
  }

  if (an_object_was_hit and closest_object_ptr) {
    closest_object_ptr->trace_object(ray);
  } else {
    if (ray->direction.y < 0) {
      get_ground_color(ray);
    } else {
      get_sky_color(ray);
      sky_was_hit = true;
    }
  }


  const Vector light_at{  0+random_offset()*30, 
                        100,
                          0+random_offset()*30};
  const Vector light_color{255,255,255};

  bool point_is_directly_lit{true};
  Ray light_probe{ray->hit_at, !(light_at - ray->hit_at), 0};
  for (const auto& object : world.scene_objects) {
    if (object->is_hit_by_ray(&light_probe)) {
      point_is_directly_lit = false;
      break;
    }
  }

  if (not sky_was_hit) {
    const float ambient_light{0.3f};
    if (point_is_directly_lit) {
      const float diffuse_light{std::max(0.f, ray->object_normal % !(light_at - ray->hit_at))};
      const float specular_factor{std::max(0.f, !(light_at - ray->hit_at) % ray->direction)};
      ray->color = ray->color * ambient_light +
                   ray->color * diffuse_light * ray->object_diffuse_factor +
                   light_color * std::pow(specular_factor, ray->object_hardness) * ray->object_specular_factor;
    } else {
      ray->color = ray->color * ambient_light;
    }
  }

  ray->color = ray->color * (1-ray->object_reflectivity);
}


void Sample(const World& world, int y, int x, int sample) 
{
  Vector ray_origin{0, 1, -4};
  Vector ray_direction = !Vector{X*(x-0.5f+random_offset()) + 
                                 Y*(y-0.5f+random_offset()) +
                                 Z};

  /// Depth-of-field
  const Vector sensor_shift{random_offset()*DoF_jitter,
                            random_offset()*DoF_jitter,
                            0};
  ray_origin = world.camera_rotation * (ray_origin + sensor_shift);
  ray_direction = !(ray_direction - sensor_shift*(1./focus_distance));
  ray_direction = world.camera_rotation * ray_direction;

  Ray* const ray = new Ray{ray_origin, 
                           ray_direction,
                           &world.raw_data[(((256-y)*512+(255+x))*AA_samples+sample)*3]};

  ray_tasks_in_flight.push_back(ray);
}

void Pixel(const World& world, int y, int x)
{
  for (int sample = 0; sample < AA_samples; ++sample) {
    Sample(world, y, x, sample);
  }
}

void Row(const World& world, int y) 
{
  /// CRITICAL to avoid RAM exhaustion!
  while (ray_tasks_in_flight.size() > 10000)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));

  ray_tasks_in_flight__mutex.lock();
  for (int x = -255; x <= 256; ++x) {
    Pixel(world, y, x);
  }
  ray_tasks_in_flight__mutex.unlock();
}

void Image(const World& world) 
{
  for (int y = 256; y >= -255; --y) {
    std::cout << std::setw(3) << 100*(256-y+1)/512 << "%\r" << std::flush;
    Row(world, y);
  }
  all_pushed = true;
}


void cleanup_worker()
{
  /// TODO account for last rays-in-flight
  while (not all_pushed) {
    if (ray_tasks_for_deletion.size()) {
      std::lock_guard<std::mutex> LOCK(ray_tasks_for_deletion__mutex);
      for (auto& ray : ray_tasks_for_deletion)
        delete ray;
      ray_tasks_for_deletion.clear();
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
}


struct WorkPackage
{
  int x_start, x_end;
  int y_start, y_end;
};
std::deque<WorkPackage> work_packages;
std::deque<WorkPackage> work_packages_done;
std::mutex work_packages__mutex;
size_t total_number_of_work_packages;


void worker_function(const World& world, int thread_idx)
{
  (void)thread_idx;

  //const int STEP{static_cast<size_t>(512/THREADS)};
  //const int y_start{std::min(256, 256-thread_idx*STEP)};
  //const int y_end{std::max(-255, 256-thread_idx*STEP-STEP)};

  while (work_packages.size()) {

    WorkPackage wp;
    {
      std::lock_guard<std::mutex> LOCK(work_packages__mutex);
      if (not work_packages.size()) 
        break;
      try {
        wp = work_packages.front();
      } catch(...) {
        break;
      }
      work_packages.pop_front();
    }

    std::deque<Ray*> thread_ray_queue;
    //std::deque<Ray const*> thread_delete_ray_queue;

    int _x, _y;
    int x{wp.x_start};
    int y{wp.y_start};
    int sample{-1};

    //while (not all_pushed) {
    while (true) {
      //if (x > 256 or y <= y_end or sample >= AA_samples)
      //  std::cout << x << " " << y << " " << sample << std::endl;
      //while (ray_tasks_in_flight.size() or
      //       thread_ray_queue.size()) {
      {
        Ray* ray;
        if (thread_ray_queue.size()) {
          ray = thread_ray_queue.front();
          thread_ray_queue.pop_front();
        } else {
          {
            ++sample;
            if (sample == AA_samples) {
              ++x;
              sample = 0;
            }
            if (x > wp.x_end) {
              ++y;
              x = wp.x_start;
            }
            if (y > wp.y_end)
              break;
          }

          _x = x - 255;
          _y = 256 - y;

          {
            Vector ray_origin{0, 1, -4};
            Vector ray_direction = !Vector{X*(_x-0.5f+random_offset()) + 
                                           Y*(_y-0.5f+random_offset()) +
                                           Z};
            /// Depth-of-field
            const Vector sensor_shift{random_offset()*DoF_jitter,
                                      random_offset()*DoF_jitter,
                                      0};
            ray_origin = world.camera_rotation * (ray_origin + sensor_shift);
            ray_direction = !(ray_direction - sensor_shift*(1./focus_distance));
            ray_direction = world.camera_rotation * ray_direction;
            ray = new Ray{ray_origin, 
                          ray_direction,
                          &world.raw_data[((y*512+x)*AA_samples+sample)*3]};
          }
        }
        ++ray_counter;

        TraceRayStep(world, ray);

        if (ray->children.size()) {
          for (auto& child : ray->children)
            /// All child rays processed by same thread;
            /// makes memory management easier
            thread_ray_queue.push_front(child);
        }

        if (ray->try_finalize()) {
          Ray const* ancestor{ray};
          while (ancestor->parent and ancestor->is_finalized)
            ancestor = ancestor->parent;

          /// Ray done?
          if ((not ancestor->parent) and ancestor->is_finalized) {

            ancestor->memory_target[0] = static_cast<unsigned char>(std::max(0.f,std::min(255.f,ancestor->color.x)));
            ancestor->memory_target[1] = static_cast<unsigned char>(std::max(0.f,std::min(255.f,ancestor->color.y)));
            ancestor->memory_target[2] = static_cast<unsigned char>(std::max(0.f,std::min(255.f,ancestor->color.z)));

            /// Yup, ray done. Delete!
            delete ancestor;
          }
        }
      }
    }

    {
      std::lock_guard<std::mutex> LOCK(work_packages__mutex);
      work_packages_done.push_back(wp);
      std::cout << " Tiles done: " << work_packages_done.size()
                << "/" << total_number_of_work_packages
                << '\r' << std::flush;
    }

  }

}


int main() {

  for (int frame = 0; frame < 1; ++frame) {

    World world;

    RotationMatrix s_rot{0,-frame/100.f*(22/7.f),0};
    world.scene_objects.push_back(new Sphere{s_rot*Vector{1,2,0}, .5});
    world.scene_objects.back()->set_color({0, 0, 0});
    world.scene_objects.back()->set_reflectivity(0.95);
    world.scene_objects.back()->set_diffuse_factor(0);
    world.scene_objects.back()->set_roughness(0.75);
    world.scene_objects.push_back(new Sphere{s_rot*Vector{-1.25,.8,0}, .25});
    world.scene_objects.back()->set_color({255, 165, 0});
    world.scene_objects.back()->set_reflectivity(0.95);
    world.scene_objects.back()->set_diffuse_factor(0.9);
    world.scene_objects.back()->set_specular_factor(1);
    world.scene_objects.back()->set_hardness(99);

    /// The octahedron has a separate rotation
    RotationMatrix o_rot{0.5f*frame/100.f*(22/7.f),
                         0.5f*frame/100.f*(22/7.f),
                         0.5f*frame/100.f*(22/7.f)};

    /// Octahedron (8 triangles)
    /// Bottom half
    world.scene_objects.push_back(new Triangle(o_rot*Vector{ 0,-1, 0}+Vector{0,1,0},
                                         o_rot*Vector{-1, 0, 0}+Vector{0,1,0},
                                         o_rot*Vector{ 0, 0, 1}+Vector{0,1,0}));
    world.scene_objects.back()->set_diffuse_factor(0);
    world.scene_objects.push_back(new Triangle(o_rot*Vector{ 0,-1, 0}+Vector{0,1,0},
                                         o_rot*Vector{ 0, 0,-1}+Vector{0,1,0},
                                         o_rot*Vector{-1, 0, 0}+Vector{0,1,0}));
    world.scene_objects.back()->set_diffuse_factor(0);
    world.scene_objects.push_back(new Triangle(o_rot*Vector{ 0,-1, 0}+Vector{0,1,0},
                                         o_rot*Vector{ 1, 0, 0}+Vector{0,1,0},
                                         o_rot*Vector{ 0, 0,-1}+Vector{0,1,0}));
    world.scene_objects.back()->set_diffuse_factor(0);
    world.scene_objects.push_back(new Triangle(o_rot*Vector{ 0,-1, 0}+Vector{0,1,0},
                                         o_rot*Vector{ 0, 0, 1}+Vector{0,1,0},
                                         o_rot*Vector{ 1, 0, 0}+Vector{0,1,0}));
    world.scene_objects.back()->set_diffuse_factor(0);
    /// Top half
    world.scene_objects.push_back(new Triangle(o_rot*Vector{ 0, 1, 0}+Vector{0,1,0},
                                         o_rot*Vector{ 0, 0, 1}+Vector{0,1,0},
                                         o_rot*Vector{-1, 0, 0}+Vector{0,1,0}));
    world.scene_objects.back()->set_diffuse_factor(0);
    world.scene_objects.push_back(new Triangle(o_rot*Vector{ 0, 1, 0}+Vector{0,1,0},
                                         o_rot*Vector{ 1, 0, 0}+Vector{0,1,0},
                                         o_rot*Vector{ 0, 0, 1}+Vector{0,1,0}));
    world.scene_objects.back()->set_diffuse_factor(0);
    world.scene_objects.push_back(new Triangle(o_rot*Vector{ 0, 1, 0}+Vector{0,1,0},
                                         o_rot*Vector{ 0, 0,-1}+Vector{0,1,0},
                                         o_rot*Vector{ 1, 0, 0}+Vector{0,1,0}));
    world.scene_objects.back()->set_diffuse_factor(0);
    world.scene_objects.push_back(new Triangle(o_rot*Vector{ 0, 1, 0}+Vector{0,1,0},
                                         o_rot*Vector{-1, 0, 0}+Vector{0,1,0},
                                         o_rot*Vector{ 0, 0,-1}+Vector{0,1,0}));
    world.scene_objects.back()->set_diffuse_factor(0);


    world.raw_data = new unsigned char[512*512*AA_samples*3];

    world.camera_rotation = {-0.2f*std::sin(frame/100.f*(22/7.f)),
                              frame/100.f*(22/7.f),
                              0.2f*std::sin(frame/100.f*(22/7.f))};

    /// Create work packages
    const int TILESIZE_X{64};
    const int TILESIZE_Y{64};
    for (int tilex = 0; tilex*TILESIZE_X < 512; ++tilex) {
      for (int tiley = 0; tiley*TILESIZE_Y < 512; ++tiley) {
        work_packages.emplace_back(WorkPackage{
          tilex*TILESIZE_X,
          std::min(512-1, (tilex+1)*TILESIZE_X),
          tiley*TILESIZE_Y,
          std::min(512-1, (tiley+1)*TILESIZE_Y)
        });
      }
    }
    total_number_of_work_packages = work_packages.size();

    std::cout << "Using " << THREADS << " render threads." << std::endl;

    std::vector<std::thread> workers;
    for (size_t thread_idx = 0; thread_idx < THREADS; ++thread_idx) {
      workers.emplace_back(std::thread{worker_function, std::ref(world), thread_idx});
    }

    /*while (work_packages_done.size() < total_number_of_work_packages) {
      std::ostringstream oss;
      /// "frame_0005.ppm"
      oss << "frame_" << std::setw(4) << std::setfill('0') << frame << ".ppm";
      std::ofstream outfile(oss.str(), std::ios::binary);

      /// PPM Header
      outfile << "P6 512 512 255 ";

      for (int y = 0; y < 512; ++y) {
        for (int x = 0; x < 512; ++x) {
          for (int c = 0; c < 3; ++c) {
            outfile << world.raw_data[(((y*512)+x)*AA_samples)*3+c];
          }
        }
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }*/

    for (auto& thread : workers)
      thread.join();

    //std::cout << "total number of rays: " << ray_counter << std::endl;


    std::ostringstream oss;
    /// "frame_0005.ppm"
    oss << "frame_" << std::setw(4) << std::setfill('0') << frame << ".ppm";
    std::ofstream outfile(oss.str(), std::ios::binary);
    std::cout << std::endl << oss.str() << std::endl;

    /// PPM Header
    outfile << "P6 512 512 255 ";

    for (int y = 0; y < 512; ++y) {
      for (int x = 0; x < 512; ++x) {
        for (int c = 0; c < 3; ++c) {
          float sum{0.f};
          for (int s = 0; s < AA_samples; ++s)
            sum += world.raw_data[(((y*512)+x)*AA_samples+s)*3+c];
          sum /= AA_samples;
          outfile << static_cast<unsigned char>(std::max(0.f,std::min(255.f,sum)));
        }
      }
    }

    if (world.raw_data)
      delete[] world.raw_data;

    for (Object* object : world.scene_objects)
      if (object)
        delete object;

  }  /// <-- frames loop end

  ///
  return EXIT_SUCCESS;
}

