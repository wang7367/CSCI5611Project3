//Vector Library [2D]
//CSCI 5611 Vector 3 Library [Incomplete]

//Instructions: Add 3D versions of all of the 2D vector functions
//              Vec3 must also support the cross product.
public class Vec3 {
  public float x, y, z;
  
  public Vec3(float x, float y, float z){
    this.x = x;
    this.y = y;
    this.z = z;
  }
  
  public float length(){
    return sqrt(x*x+y*y+z*z);
  }
  
  public Vec3 plus(Vec3 rhs){
    return new Vec3(x+rhs.x,y+rhs.y,z+rhs.z);
  }
  
  public void add(Vec3 rhs){
    x += rhs.x;
    y += rhs.y;
    z += rhs.z;
  }
  
  public Vec3 minus(Vec3 rhs){
    return new Vec3(x-rhs.x,y-rhs.y,z-rhs.z);
  }
  
  public void subtract(Vec3 rhs){
    x -= rhs.x;
    y -= rhs.y;
    z -= rhs.z;
  }
  
  public Vec3 times(float rhs){
    return new Vec3(rhs*x, rhs*y, rhs*z);
  }
  
  public void normalize(){
    float vecLen = this.length();
    x /= vecLen;
    y /= vecLen;
    z /= vecLen;
  }
}

Vec3 interpolate(Vec3 a, Vec3 b, float t){
  return a.plus(b.minus(a).times(t)); 
}

Vec3 cross(Vec3 a, Vec3 b){
  return new Vec3(a.y*b.z-b.y*a.z,
                  b.x*a.z-b.z*a.x,
                  a.x*b.y-b.x*a.y);
}
