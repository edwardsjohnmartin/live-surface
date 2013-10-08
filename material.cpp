#include <string>
#include "./material.h"
#include "./texture.h"

using namespace std;

Material::Material()
    : _ambient(Vec3f(.3, .3, .3)),
      _diffuse(Vec3f(.7, .7, .7)),
      _specular(Vec3f(.1, .1, .1)), _specular_coeff(10),
      _texture_id(-1) {
}

Material::Material(const string& name)
    : _name(name),
      _ambient(Vec3f(.1, .1, .1)),
      _diffuse(Vec3f(.3, .3, .3)),
      _specular(Vec3f(.1, .1, .1)), _specular_coeff(10),
      _texture_id(-1) {
}

Material Material::FromDiffuseAmbient(const Vec3f& diff_amb, float coeff) {
  Material m;
  m.set_diffuse(diff_amb);
  m.set_ambient(diff_amb);
  // m.set_specular(diff_amb);
  m.set_specular(Vec3f(1, 1, 1));
  m.set_specular_coeff(coeff);
  return m;
}

void Material::set_ambient(const Vec3f& ambient) {
  _ambient = ambient;
}
void Material::set_diffuse(const Vec3f& diffuse) {
  _diffuse = diffuse;
}
void Material::set_specular(const Vec3f& specular) {
  _specular = specular;
}
void Material::set_specular_coeff(const float& coeff) {
  _specular_coeff = coeff;
}
void Material::set_texture(const string& texture) {
  _texture = texture;
}

void Material::LoadTexture(int texture_id) {
  ::LoadTexture(texture(), texture_id);
  _texture_id = texture_id;
}
