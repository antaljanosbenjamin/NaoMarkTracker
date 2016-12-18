#ifndef MY_VECTOR_H
#define MY_VECTOR_H

struct Vector {
	float x, y, z;

	Vector() {
		x = y = z = 0;
	}
	Vector(float x0, float y0, float z0 = 0) {
		x = x0; y = y0; z = z0;
	}
	Vector operator*(float a) {
		return Vector(x * a, y * a, z * a);
	}
	Vector operator+(const Vector v) {
		return Vector(x + v.x, y + v.y, z + v.z);
	}
	Vector operator-(const Vector v) {
		return Vector(x - v.x, y - v.y, z - v.z);
	}
	float operator*(const Vector& v) { 	// dot product
		return (x * v.x + y * v.y + z * v.z);
	}
	Vector operator/(float a){
		return Vector(x / a, y / a, z / a);
	}

	Vector operator%(const Vector v) { 	// cross product
		return Vector(y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x);
	}

	float Length() { return sqrt(x * x + y * y + z * z); };

	float getDistance(Vector v){
		return pow((pow(x - v.x, 2) + pow(y - v.y, 2) + pow(z - v.z, 2)), 0.5f);
	};
	Vector normalize(){
		return (*this) / this->Length();
	};

	Vector rotateByX(float fi){
		return  Vector(x, y * cos(fi) - z * sin(fi), y * sin(fi) + z * cos(fi));
	};

	Vector rotateByY(float fi){
		return  Vector(x * cos(fi) + z * sin(fi), y, - x * sin(fi) + z * cos(fi));
	};

	Vector rotateByZ(float fi){
		return  Vector(x * cos(fi) - y * sin(fi), x * sin(fi) + y * cos(fi), z);
	};

	
};
#endif
