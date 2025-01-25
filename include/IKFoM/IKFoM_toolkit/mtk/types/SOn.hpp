// This is an advanced implementation of the algorithm described in the
// following paper:
//    C. Hertzberg,  R.  Wagner,  U.  Frese,  and  L.  Schroder.  Integratinggeneric   sensor   fusion   algorithms   with   sound   state   representationsthrough  encapsulation  of  manifolds.
//    CoRR,  vol.  abs/1107.1119,  2011.[Online]. Available: http://arxiv.org/abs/1107.1119

/*
 *  Copyright (c) 2019--2023, The University of Hong Kong
 *  All rights reserved.
 *
 *  Modifier: Dongjiao HE <hdj65822@connect.hku.hk>
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Universitaet Bremen nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
 
/*
 *  Copyright (c) 2008--2011, Universitaet Bremen
 *  All rights reserved.
 *
 *  Author: Christoph Hertzberg <chtz@informatik.uni-bremen.de>
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Universitaet Bremen nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * @file mtk/types/SOn.hpp
 * @brief Standard Orthogonal Groups i.e.\ rotatation groups.
 */
#ifndef SON_H_
#define SON_H_

#include <Eigen/Geometry>

#include "vect.hpp"
#include "../src/mtkmath.hpp"


namespace MTK {


/**
 * Two-dimensional orientations represented as scalar.
 * There is no guarantee that the representing scalar is within any interval,
 * but the result of boxminus will always have magnitude @f$\le\pi @f$.
 */
template<class _scalar = double, int Options = Eigen::AutoAlign>
struct SO2 : public Eigen::Rotation2D<_scalar> {
	enum {DOF = 1, DIM = 2, TYP = 3};
	
	typedef _scalar scalar;
	typedef Eigen::Rotation2D<scalar> base;
	typedef vect<DIM, scalar, Options> vect_type;
	
	//! Construct from angle
	SO2(const scalar& angle = 0) : base(angle) {	}
	
	//! Construct from Eigen::Rotation2D
	SO2(const base& src) : base(src) {}
	
	/**
	 * Construct from 2D vector.
	 * Resulting orientation will rotate the first unit vector to point to vec.
	 */
	SO2(const vect_type &vec) : base(atan2(vec[1], vec[0])) {};
	
	
	//! Calculate @c this->inverse() * @c r
	SO2 operator%(const base &r) const {
		return base::inverse() * r;
	}

	//! Calculate @c this->inverse() * @c r
	template<class Derived>
	vect_type operator%(const Eigen::MatrixBase<Derived> &vec) const {
		return base::inverse() * vec;
	}
	
	//! Calculate @c *this * @c r.inverse()
	SO2 operator/(const SO2 &r) const {
		return *this * r.inverse();
	}
	
	//! Gets the angle as scalar.
	operator scalar() const {
		return base::angle(); 
	}
	void S2_hat(Eigen::Matrix<scalar, 3, 3> &res)
	{
		res = Eigen::Matrix<scalar, 3, 3>::Zero();
	}
	//! @name Manifold requirements
	void S2_Nx_yy(Eigen::Matrix<scalar, 2, 3> &res)
	{
		std::cerr << "wrong idx for S2" << std::endl;
		std::exit(100);	
    	res = Eigen::Matrix<scalar, 2, 3>::Zero();
	}

	void S2_Mx(Eigen::Matrix<scalar, 3, 2> &res, MTK::vectview<const scalar, 2> delta)
	{
		std::cerr << "wrong idx for S2" << std::endl;
		std::exit(100);	
    	res = Eigen::Matrix<scalar, 3, 2>::Zero();
	}

	void oplus(MTK::vectview<const scalar, DOF> vec, scalar scale = 1) {
		base::angle() += scale * vec[0];
	}
	
	void boxplus(MTK::vectview<const scalar, DOF> vec, scalar scale = 1) {
		base::angle() += scale * vec[0];
	}
	void boxminus(MTK::vectview<scalar, DOF> res, const SO2<scalar>& other) const {
		res[0] = MTK::normalize(base::angle() - other.angle(), scalar(MTK::pi));
	}
	
	friend std::istream& operator>>(std::istream &is, SO2<scalar>& ang){
		return is >> ang.angle();
	}
	void hat(Eigen::VectorXd& v, Eigen::MatrixXd &res) {
		std::cout << "wrong idx" << std::endl;
	}
	void Jacob_right_inv(Eigen::VectorXd& v, Eigen::MatrixXd &res) {
		std::cout << "wrong idx" << std::endl;
	}
	void Jacob_right(Eigen::VectorXd& v, Eigen::MatrixXd &res) {
		std::cout << "wrong idx" << std::endl;
	}
};


/**
 * Three-dimensional orientations represented as Quaternion.
 * It is assumed that the internal Quaternion always stays normalized,
 * should this not be the case, call inherited member function @c normalize().
 */
template<class _scalar = double> //, int Options = Eigen::AutoAlign>
struct SO3 : public Eigen::Matrix<_scalar, 3, 3> {
	enum {DOF = 3, DIM = 3, TYP = 2}; 
	typedef _scalar scalar;
	typedef Eigen::Matrix<scalar, 3, 3> base;
	typedef Eigen::Matrix<scalar, 3, 3> Matrix;
	typedef vect<DIM, scalar> vect_type;
	
	/**
	 * Construct from Eigen::Quaternion.
	 * @note Non-normalized input may result result in spurious behavior.
	 */
	SO3(const base& src = base::Identity()) : base(src) {}
	
	/**
	 * Construct from rotation matrix.
	 * @note Invalid rotation matrices may lead to spurious behavior.
	 */
	template<class Derived>
	SO3(const Eigen::MatrixBase<Derived>& matrix) : base(matrix) {}
	
	/**
	 * Construct from arbitrary rotation type.
	 * @note Invalid rotation matrices may lead to spurious behavior.
	 */
	// template<class Derived>
	// SO3(const Eigen::RotationBase<Derived, 3>& rotation) : base(rotation.derived()) {}
	
	//! @name Manifold requirements
	
	// SO3 operator=(const base &r) const {
	// 	return r;
	// }

	// //! Calculate @c *this * @c r.inverse()
	// SO3 operator*(const SO3 &r) const {
	// 	return *this * r;
	// }

	void boxplus(MTK::vectview<const scalar, DOF> vec, scalar scale=1) {
		SO3 delta = exp(vec, scale);
		*this = *this * delta;
	}
	void boxminus(MTK::vectview<scalar, DOF> res, const SO3<scalar>& other) const {
		res = SO3::log(other.transpose() * *this);
	}
	//}

	void oplus(MTK::vectview<const scalar, DOF> vec, scalar scale=1) {
		SO3 delta = exp(vec, scale);
		*this = *this * delta;
	}

	// void hat(MTK::vectview<const scalar, DOF>& v, Eigen::Matrix<scalar, 3, 3> &res) {
	void hat(Eigen::VectorXd& v, Eigen::MatrixXd &res) {
		Eigen::Matrix<double, 3, 3> hatv;
		hatv << 0, -v[2], v[1],
			v[2], 0, -v[0],
			-v[1], v[0], 0;
		res = hatv;
	}

	// void Jacob_right_inv(MTK::vectview<const scalar, DOF> vec, Eigen::Matrix<scalar, 3, 3> & res){
	void Jacob_right_inv(Eigen::VectorXd& vec, Eigen::MatrixXd &res){
    	Eigen::MatrixXd hat_v;
		hat(vec, hat_v);
    	if(vec.norm() > MTK::tolerance<scalar>())
    	{
        	res = Eigen::Matrix<scalar, 3, 3>::Identity() + 0.5 * hat_v + (1 - vec.norm() * std::cos(vec.norm() / 2) / 2 / std::sin(vec.norm() / 2)) * hat_v * hat_v / vec.squaredNorm();
    	}
    	else
    	{
        	res = Eigen::Matrix<scalar, 3, 3>::Identity();
    	}
    	// return res;
	}

	// void Jacob_right(MTK::vectview<const scalar, DOF> & v, Eigen::Matrix<scalar, 3, 3> &res){
	void Jacob_right(Eigen::VectorXd& v, Eigen::MatrixXd &res){
		Eigen::MatrixXd hat_v;
		hat(v, hat_v);
		double squaredNorm = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
		double norm = std::sqrt(squaredNorm);
		if(norm < MTK::tolerance<scalar>()){
			res = Eigen::Matrix<scalar, 3, 3>::Identity();
		}
		else{
			res = Eigen::Matrix<scalar, 3, 3>::Identity() - (1 - std::cos(norm)) / squaredNorm * hat_v + (1 - std::sin(norm) / norm) / squaredNorm * hat_v * hat_v;
		}
		// return res;
	}

	void S2_hat(Eigen::Matrix<scalar, 3, 3> &res)
	{
		res = Eigen::Matrix<scalar, 3, 3>::Zero();
	}
	void S2_Nx_yy(Eigen::Matrix<scalar, 2, 3> &res)
	{
		std::cerr << "wrong idx for S2" << std::endl;
		std::exit(100);	
    	res = Eigen::Matrix<scalar, 2, 3>::Zero();
	}

	void S2_Mx(Eigen::Matrix<scalar, 3, 2> &res, MTK::vectview<const scalar, 2> delta)
	{
		std::cerr << "wrong idx for S2" << std::endl;
		std::exit(100);	
    	res = Eigen::Matrix<scalar, 3, 2>::Zero();
	}

	friend std::ostream& operator<<(std::ostream &os, const SO3<scalar>& q){ // wrong!
		return os << q.data() << " ";
	}

	friend std::istream& operator>>(std::istream &is, SO3<scalar>& q){ // wrong!
		SO3<scalar> coeffs;
		is >> coeffs;
		q = coeffs;
		return is;
	}
	
	//! @name Helper functions
	//{
	/**
	 * Calculate the exponential map. In matrix terms this would correspond 
	 * to the Rodrigues formula.
	 */
	// FIXME vectview<> can't be constructed from every MatrixBase<>, use const Vector3x& as workaround
//	static SO3 exp(MTK::vectview<const scalar, 3> dvec, scalar scale = 1){
	static SO3 exp(const Eigen::Matrix<scalar, 3, 1>& dvec, scalar scale = 1){
		Eigen::Matrix<scalar, 3, 1> ang = dvec * scale;
		double ang_norm = ang.norm();
    	Eigen::Matrix<double, 3, 3> Eye3 = Eigen::Matrix<double, 3, 3>::Identity();
    	if (ang_norm > 0.0000001)
    	{
        	Eigen::Matrix<double, 3, 1> r_axis = ang / ang_norm;
        	Eigen::Matrix<double, 3, 3> K;
        	K << 0.0, -r_axis(2), r_axis(1),
				r_axis(2), 0.0, -r_axis(0),
				-r_axis(1), r_axis(0), 0.0; // SKEW_SYM_MATRX(r_axis);
        	/// Roderigous Tranformation
        	return Eye3 + std::sin(ang_norm) * K + (1.0 - std::cos(ang_norm)) * K * K;
    	}
    	else
    	{
        	return Eye3;
    	}
	}
	/**
	 * Calculate the inverse of @c exp.
	 * Only guarantees that <code>exp(log(x)) == x </code>
	 */
	static Eigen::Vector3d log(const SO3 &orient){
		double theta = (orient.trace() > 3.0 - 1e-6) ? 0.0 : std::acos(0.5 * (orient.trace() - 1));
    	Eigen::Matrix<double,3,1> K(orient(2,1) - orient(1,2), orient(0,2) - orient(2,0), orient(1,0) - orient(0,1));
    	return (std::abs(theta) < 0.001) ? (0.5 * K) : (0.5 * theta / std::sin(theta) * K);
	}
};

namespace internal {
template<class Scalar, int Options>
struct UnalignedType<SO2<Scalar, Options > >{
	typedef SO2<Scalar, Options | Eigen::DontAlign> type;
};

template<class Scalar>
struct UnalignedType<SO3<Scalar> >{
	typedef SO3<Scalar> type;
};

}  // namespace internal


}  // namespace MTK

#endif /*SON_H_*/

