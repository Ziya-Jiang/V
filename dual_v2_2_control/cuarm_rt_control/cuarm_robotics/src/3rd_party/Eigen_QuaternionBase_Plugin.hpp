/** \returns Great circle distance from other quaternion to this one

*/
template<class OtherDerived> EIGEN_DEVICE_FUNC Scalar greatcircleDistance(const QuaternionBase<OtherDerived>& other) const
{
  EIGEN_USING_STD(acos)
  Quaternion<Scalar> d = selectClosest((*this) * other.conjugate());
  return acos(d.w())*2.0;
}

/** \returns axis of rotation between the orientation represented by two quaternions 
 * 
 * Givin a unit quaternion q (used to rotate vector v) it can be decomposed
into subcomponents (a, b*u) where a and b are scalars and u is a unit vector.
u is the axis of the rotation given by q and theta is the angle of rotation.
a = cos(theta/2) ; b = sin(theta/2) 
 */
template<class OtherDerived> EIGEN_DEVICE_FUNC Matrix<Scalar,3,1> greatcircleAxis( const QuaternionBase<OtherDerived>& other) const
{
    Quaternion<Scalar> d = selectClosest((*this) * other.conjugate());
    Matrix<Scalar,3,1> ret;
    ret <<  d.x(), d.y(), d.z() ;
    ret.normalize();
    return ret;
}

/** \returns If this quaternion represents a rotation > pi, change it to the equivalent quaternion 
 * representing a rotation < pi to the same point on the unit sphere
 */
template<class OtherDerived> EIGEN_DEVICE_FUNC Quaternion<Scalar> selectClosest( const QuaternionBase<OtherDerived>& other) const
{
    EIGEN_USING_STD(cos)
    EIGEN_USING_STD(acos)
    Quaternion<Scalar> ret;
    
    Scalar flip = 1.0;
    Scalar tmp_w = other.w();
    
    // tmp_w may be > 1 due to roundoff and cause problems with acos
    
    if ( tmp_w > 1.0 ) {
        tmp_w = 1.0;
    }
    else if ( tmp_w < -1.0 ) {
        tmp_w = -1.0;
    }

    
    if ( tmp_w < 0.0 ) {
        flip = -1.0;
        tmp_w = cos(EIGEN_PI - acos( tmp_w ));
    }
    
    ret.x() = other.x() * flip;
    ret.y() = other.y() * flip;
    ret.z() = other.z() * flip;
    ret.w() = tmp_w;
    

    return ret;
}
