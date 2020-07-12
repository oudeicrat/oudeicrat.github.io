/*
 * https://fossies.org/linux/privat/gfsview-snapshot-121130.tar.gz/gfsview-snapshot-121130/gl/trackball.c?m=t
 * Trackball code:
 *
 * Implementation of a virtual trackball.
 * Implemented by Gavin Bell, lots of ideas from Thant Tessman and
 *   the August '88 issue of Siggraph's "Computer Graphics," pp. 121-129.
 *
 * Vector manip code:
 *
 * Original code from:
 * David M. Ciemiewicz, Mark Grossman, Henry Moreton, and Paul Haeberli
 *
 * Much mucking with by:
 * Gavin Bell
 */

/*
 * This size should really be based on the distance from the center of
 * rotation to the point on the object underneath the mouse.  That
 * point would then track the mouse as closely as possible.  This is a
 * simple example, though, so that is left as an Exercise for the
 * Programmer.
 */
var TRACKBALLSIZE = 0.3;

function vzero() {
	var v = [0,0,0];
	return v;
}

function vsub(src1, src2) {
	var dst = [0,0,0];
	dst[0] = src1[0] - src2[0];
	dst[1] = src1[1] - src2[1];
	dst[2] = src1[2] - src2[2];
	return dst;
}

function vcopy(v) {
	var result = [0,0,0];
	for (var i = 0; i < 3; i++)
		result[i] = v[i];
	return result;
}

function vcross(v1, v2)
{
	var result = [0,0,0];
	result[0] = (v1[1] * v2[2]) - (v1[2] * v2[1]);
	result[1] = (v1[2] * v2[0]) - (v1[0] * v2[2]);
	result[2] = (v1[0] * v2[1]) - (v1[1] * v2[0]);
	return result;
}

function vlength(v) {
	return Math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

function vscale(v, div) {
	v[0] *= div;
	v[1] *= div;
	v[2] *= div;
	return v;
}

function vnormal(v) {
	return vscale(v,1/vlength(v));
}

function vdot(v1, v2) {
	return v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2];
}

function vadd(src1, src2) {
	var dst = [0,0,0,0];
	dst[0] = src1[0] + src2[0];
	dst[1] = src1[1] + src2[1];
	dst[2] = src1[2] + src2[2];
	return dst;
}

/*
 * Ok, simulate a track-ball.  Project the points onto the virtual
 * trackball, then figure out the axis of rotation, which is the cross
 * product of P1 P2 and O P1 (O is the center of the ball, 0,0,0)
 * Note:  This is a deformed trackball-- is a trackball in the center,
 * but is deformed into a hyperbolic sheet of rotation away from the
 * center.  This particular function was chosen after trying out
 * several variations.
 *
 * It is assumed that the arguments to this routine are in the range
 * (-1.0 ... 1.0)
 */
function gfs_gl_trackball(p1x, p1y, p2x, p2y) {
	var phi;

	if (p1x == p2x && p1y == p2y) {
		//Zero rotation
		return [0,0,0,1];
	}

	//First, figure out z-coordinates for projection of P1 and P2 to deformed sphere
	p1 = [p1x,p1y,tb_project_to_sphere(TRACKBALLSIZE,p1x,p1y)];
	p2 = [p2x,p2y,tb_project_to_sphere(TRACKBALLSIZE,p2x,p2y)];

	//Now, we want the cross product of P1 and P2
 	var a = vcross(p2,p1); //axis of rotation

	//Figure out how much to rotate around that axis.
	var d = vsub(p1,p2);
	var t = vlength(d) / (2.0*TRACKBALLSIZE);

	//Avoid problems with out-of-control values...
	if (t > 1.0) t = 1.0;
	if (t < -1.0) t = -1.0;
	var phi = 2.0 * Math.asin(t); //how much to rotate about axis

	return gfs_gl_axis_to_quat(a,phi);
}

//Given an axis and angle, compute quaternion.
function gfs_gl_axis_to_quat(a, phi) {
	a = vnormal(a);
	var q = vcopy(a,q);
	q = vscale(q, Math.sin(phi/2.0));
	q[3] = Math.cos(phi/2.0);
	return q;
}

//Project an x,y pair onto a sphere of radius r OR a hyperbolic sheet
//if we are away from the center of the sphere.
function tb_project_to_sphere(r, x, y) {
	var z = 0;

	var d = Math.sqrt(x*x + y*y);
	if (d < r * 0.70710678118654752440) { //Inside sphere
		z = Math.sqrt(r*r - d*d);
	} else {
		//On hyperbola
		var t = r / 1.41421356237309504880;
		z = t*t / d;
	}
	return z;
}

/*
 * Given two rotations, e1 and e2, expressed as quaternion rotations,
 * figure out the equivalent single rotation and stuff it into dest.
 *
 * This routine also normalizes the result every RENORMCOUNT times it is
 * called, to keep error from creeping in.
 *
 * NOTE: This routine is written so that q1 or q2 may be the same
 * as dest (or each other).
 */

var RENORMCOUNT = 200;
var gfs_gl_add_quats_count = 0;

function gfs_gl_add_quats(q1, q2) {
	var result = [
		q1[0]*q2[3] + q2[0]*q1[3] + q2[1]*q1[2] - q2[2]*q1[1],
		q1[1]*q2[3] + q2[1]*q1[3] + q2[2]*q1[0] - q2[0]*q1[2],
		q1[2]*q2[3] + q2[2]*q1[3] + q2[0]*q1[1] - q2[1]*q1[0],
		q1[3]*q2[3] - q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2]
	];

	if (++gfs_gl_add_quats_count > RENORMCOUNT) {
		gfs_gl_add_quats_count = 0;
		//console.log('renormalizing');
		result = normalize_quat(result);
	}
	return result;
}

/*
 * Quaternions always obey:  a^2 + b^2 + c^2 + d^2 = 1.0
 * If they don't add up to 1.0, dividing by their magnitued will
 * renormalize them.
 *
 * Note: See the following for more information on quaternions:
 *
 * - Shoemake, K., Animating rotation with quaternion curves, Computer
 *   Graphics 19, No 3 (Proc. SIGGRAPH'85), 245-254, 1985.
 * - Pletinckx, D., Quaternion calculus as a basic tool in computer
 *   graphics, The Visual Computer 5, 2-13, 1989.
 */
function normalize_quat(q) {
	var mag = (q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]); //original c code
	//var mag = Math.sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]); //sqrt is not needed if the distortion is not too great
	for (var i = 0; i < 4; i++) q[i] /= mag;
	return q;
}

//Build a rotation matrix, given a quaternion rotation.
function gfs_gl_build_rotmatrix(q) {
	var m = [[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,1]];
	m[0][0] = 1 - 2 * (q[1] * q[1] + q[2] * q[2]);
	m[0][1] = 2 * (q[0] * q[1] - q[2] * q[3]);
	m[0][2] = 2 * (q[2] * q[0] + q[1] * q[3]);
	m[0][3] = 0;

	m[1][0] = 2 * (q[0] * q[1] + q[2] * q[3]);
	m[1][1] = 1 - 2 * (q[2] * q[2] + q[0] * q[0]);
	m[1][2] = 2 * (q[1] * q[2] - q[0] * q[3]);
	m[1][3] = 0;

	m[2][0] = 2 * (q[2] * q[0] - q[1] * q[3]);
	m[2][1] = 2 * (q[1] * q[2] + q[0] * q[3]);
	m[2][2] = 1 - 2 * (q[1] * q[1] + q[0] * q[0]);
	m[2][3] = 0;

	m[3][0] = 0;
	m[3][1] = 0;
	m[3][2] = 0;
	m[3][3] = 1;
	return m;
}
