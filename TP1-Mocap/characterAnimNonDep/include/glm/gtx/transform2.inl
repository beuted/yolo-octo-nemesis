///////////////////////////////////////////////////////////////////////////////////////////////////
// OpenGL Mathematics Copyright (c) 2005 - 2010 G-Truc Creation (www.g-truc.net)
///////////////////////////////////////////////////////////////////////////////////////////////////
// Created : 2005-02-28
// Updated : 2005-04-23
// Licence : This source is under MIT License
// File : glm/gtx/transform2.inl
///////////////////////////////////////////////////////////////////////////////////////////////////

namespace glm{
namespace gtx{
namespace transform2
{
    template <typename T> 
    inline detail::tmat3x3<T> shearX2D(
		const detail::tmat3x3<T>& m, 
		T s)
    {
        detail::tmat3x3<T> r(1);
        r[0][1] = s;
        return m * r;
    }

    template <typename T> 
    inline detail::tmat3x3<T> shearY2D(
		const detail::tmat3x3<T>& m, 
		T s)
    {
        detail::tmat3x3<T> r(1);
        r[1][0] = s;
        return m * r;
    }

    template <typename T> 
    inline detail::tmat4x4<T> shearX3D(
		const detail::tmat4x4<T>& m, 
		T s, 
		T t)
    {
        detail::tmat4x4<T> r(1);
        r[1][0] = s;
        r[2][0] = t;
        return m * r;
    }

    template <typename T> 
    inline detail::tmat4x4<T> shearY3D(
		const detail::tmat4x4<T>& m, 
		T s, 
		T t)
    {
        detail::tmat4x4<T> r(1);
        r[0][1] = s;
        r[2][1] = t;
        return m * r;
    }

    template <typename T>     
	inline detail::tmat4x4<T> shearZ3D(
		const detail::tmat4x4<T>& m, 
		T s, 
		T t)
    {
        detail::tmat4x4<T> r(1);
        r[0][2] = s;
        r[1][2] = t;
        return m * r;
    }

    template <typename T> 
    inline detail::tmat3x3<T> reflect2D(
		const detail::tmat3x3<T>& m, 
		const detail::tvec3<T>& normal)
    {
        detail::tmat3x3<T> r(1);
        r[0][0] = 1 - 2 * normal.x * normal.x;
        r[0][1] = -2 * normal.x * normal.y;
        r[1][0] = -2 * normal.x * normal.y;
        r[1][1] = 1 - 2 * normal.y * normal.y;
        return m * r;
    }

    template <typename T> 
    inline detail::tmat4x4<T> reflect3D(
		const detail::tmat4x4<T>& m, 
		const detail::tvec3<T>& normal)
    {
        detail::tmat4x4<T> r(1);
        r[0][0] = 1 - 2 * normal.x * normal.x;
        r[0][1] = -2 * normal.x * normal.y;
        r[0][2] = -2 * normal.x * normal.z;

        r[1][0] = -2 * normal.x * normal.y;
        r[1][1] = 1 - 2 * normal.y * normal.y;
        r[1][2] = -2 * normal.y * normal.z;

        r[2][0] = -2 * normal.x * normal.z;
        r[2][1] = -2 * normal.y * normal.z;
        r[2][2] = 1 - 2 * normal.z * normal.z;
        return m * r;
    }

    template <typename T> 
    inline detail::tmat3x3<T> proj2D(
		const detail::tmat3x3<T>& m, 
		const detail::tvec3<T>& normal)
    {
        detail::tmat3x3<T> r(1);
        r[0][0] = 1 - normal.x * normal.x;
        r[0][1] = - normal.x * normal.y;
        r[1][0] = - normal.x * normal.y;
        r[1][1] = 1 - normal.y * normal.y;
        return m * r;
    }

    template <typename T> 
    inline detail::tmat4x4<T> proj3D(
		const detail::tmat4x4<T>& m, 
		const detail::tvec3<T>& normal)
    {
        detail::tmat4x4<T> r(1);
        r[0][0] = 1 - normal.x * normal.x;
        r[0][1] = - normal.x * normal.y;
        r[0][2] = - normal.x * normal.z;
        r[1][0] = - normal.x * normal.y;
        r[1][1] = 1 - normal.y * normal.y;
        r[1][2] = - normal.y * normal.z;
        r[2][0] = - normal.x * normal.z;
        r[2][1] = - normal.y * normal.z;
        r[2][2] = 1 - normal.z * normal.z;
        return m * r;
    }

    template <typename T> 
    inline detail::tmat4x4<T> lookAt(
		const detail::tvec3<T>& eye, 
		const detail::tvec3<T>& center, 
		const detail::tvec3<T>& up)
    {
        detail::tvec3<T> f = normalize(center - eye);
        detail::tvec3<T> u = normalize(up);
        detail::tvec3<T> s = normalize(cross(f, u));
        u = cross(s, f);

        detail::tmat4x4<T> Result(1);
        Result[0][0] = s.x;
        Result[1][0] = s.y;
        Result[2][0] = s.z;
        Result[0][1] = u.x;
        Result[1][1] = u.y;
        Result[2][1] = u.z;
        Result[0][2] =-f.x;
        Result[1][2] =-f.y;
        Result[2][2] =-f.z;
    /*  Test this instead of translate3D
        Result[3][0] =-dot(s, eye);
        Result[3][1] =-dot(y, eye);
        Result[3][2] = dot(f, eye);
    */  
		return gtc::matrix_transform::translate(Result, -eye);
    }

	template <typename T> 
	inline detail::tmat4x4<T> scaleBias(
		T scale, 
		T bias)
	{
		detail::tmat4x4<T> result;
		result[3] = detail::tvec4<T>(detail::tvec3<T>(bias), T(1));
		result[0][0] = scale;
		result[1][1] = scale;
		result[2][2] = scale;
		return result;
	}

	template <typename T> 
	inline detail::tmat4x4<T> scaleBias(
		const detail::tmat4x4<T>& m, 
		T scale, 
		T bias)
	{
		return m * scaleBias(scale, bias);
	}

}//namespace transform2
}//namespace gtx
}//namespace glm

