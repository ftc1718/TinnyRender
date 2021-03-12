//#include <vector>
//#include <limits>
//#include <iostream>
//#include <algorithm>
//#include "tgaimage.h"
//#include "model.h"
//#include "geometry.h"
//#include "our_gl.h"
//
//Model *model = NULL;
//float *shadowbuffer = NULL;
//
//const int width = 800;
//const int height = 800;
//
//Vec3f light_dir(1, 1, 0);
//Vec3f       eye(1, 1, 4);
//Vec3f    center(0, 0, 0);
//Vec3f        up(0, 1, 0);
//
//struct Shader : public IShader {
//	mat<4, 4, float> uniform_M;   //  Projection*ModelView
//	mat<4, 4, float> uniform_MIT; // (Projection*ModelView).invert_transpose()
//	mat<4, 4, float> uniform_Mshadow; // transform framebuffer screen coordinates to shadowbuffer screen coordinates
//	mat<2, 3, float> varying_uv;  // triangle uv coordinates, written by the vertex shader, read by the fragment shader
//	mat<3, 3, float> varying_tri; // triangle coordinates before Viewport transform, written by VS, read by FS
//
//	Shader(Matrix M, Matrix MIT, Matrix MS) : uniform_M(M), uniform_MIT(MIT), uniform_Mshadow(MS), varying_uv(), varying_tri() {}
//
//	virtual Vec4f vertex(int iface, int nthvert) {
//		varying_uv.set_col(nthvert, model->uv(iface, nthvert));
//		Vec4f gl_Vertex = Viewport * Projection*ModelView*embed<4>(model->vert(iface, nthvert));
//		varying_tri.set_col(nthvert, proj<3>(gl_Vertex / gl_Vertex[3]));
//		return gl_Vertex;
//	}
//
//	virtual bool fragment(Vec3f bar, TGAColor &color) {
//		Vec4f sb_p = uniform_Mshadow * embed<4>(varying_tri*bar); // corresponding point in the shadow buffer
//		sb_p = sb_p / sb_p[3];
//		int idx = int(sb_p[0]) + int(sb_p[1])*width; // index in the shadowbuffer array
//		float shadow = .3 + .7*(shadowbuffer[idx] < sb_p[2] + 43.34); // magic coeff to avoid z-fighting
//		Vec2f uv = varying_uv * bar;                 // interpolate uv for the current pixel
//		Vec3f n = proj<3>(uniform_MIT*embed<4>(model->normal(uv))).normalize(); // normal
//		Vec3f l = proj<3>(uniform_M  *embed<4>(light_dir)).normalize(); // light vector
//		Vec3f r = (n*(n*l*2.f) - l).normalize();   // reflected light
//		float spec = pow(std::max(r.z, 0.0f), model->specular(uv));
//		float diff = std::max(0.f, n*l);
//		TGAColor c = model->diffuse(uv);
//		for (int i = 0; i < 3; i++) color[i] = std::min<float>(20 + c[i] * shadow*(1.2*diff + .6*spec), 255);
//		return false;
//	}
//};
//
//struct DepthShader : public IShader {
//	mat<3, 3, float> varying_tri;
//
//	DepthShader() : varying_tri() {}
//
//	virtual Vec4f vertex(int iface, int nthvert) {
//		Vec4f gl_Vertex = embed<4>(model->vert(iface, nthvert)); // read the vertex from .obj file
//		gl_Vertex = Viewport * Projection*ModelView*gl_Vertex;          // transform it to screen coordinates
//		varying_tri.set_col(nthvert, proj<3>(gl_Vertex / gl_Vertex[3]));
//		return gl_Vertex;
//	}
//
//	virtual bool fragment(Vec3f bar, TGAColor &color) {
//		Vec3f p = varying_tri * bar;
//		color = TGAColor(255, 255, 255)*(p.z / depth);
//		return false;
//	}
//};
//
//int main(int argc, char** argv) {
//    if (2==argc) {
//        model = new Model(argv[1]);
//    } else {
//        model = new Model("obj/diablo3_pose/diablo3_pose.obj");
//    }
//
//	float *zbuffer = new float[width*height];
//	shadowbuffer = new float[width*height];
//	for (int i = width * height; --i; ) {
//		zbuffer[i] = shadowbuffer[i] = -std::numeric_limits<float>::max();
//	}
//
//	light_dir.normalize();
//
//	{ // rendering the shadow buffer
//		TGAImage depth(width, height, TGAImage::RGB);
//		lookat(light_dir, center, up);
//		viewport(width / 8, height / 8, width * 3 / 4, height * 3 / 4);
//		projection(0);
//
//		DepthShader depthshader;
//		Vec4f screen_coords[3];
//		for (int i = 0; i < model->nfaces(); i++) {
//			for (int j = 0; j < 3; j++) {
//				screen_coords[j] = depthshader.vertex(i, j);
//			}
//			triangle(screen_coords, depthshader, depth, shadowbuffer);
//		}
//		depth.flip_vertically(); // to place the origin in the bottom left corner of the image
//		depth.write_tga_file("depth.tga");
//	}
//
//	Matrix M = Viewport * Projection*ModelView;
//
//	{ // rendering the frame buffer
//		TGAImage frame(width, height, TGAImage::RGB);
//		lookat(eye, center, up);
//		viewport(width / 8, height / 8, width * 3 / 4, height * 3 / 4);
//		projection(-1.f / (eye - center).norm());
//
//		Shader shader(ModelView, (Projection*ModelView).invert_transpose(), M*(Viewport*Projection*ModelView).invert());
//		Vec4f screen_coords[3];
//		for (int i = 0; i < model->nfaces(); i++) {
//			for (int j = 0; j < 3; j++) {
//				screen_coords[j] = shader.vertex(i, j);
//			}
//			triangle(screen_coords, shader, frame, zbuffer);
//		}
//		frame.flip_vertically(); // to place the origin in the bottom left corner of the image
//		frame.write_tga_file("framebuffer.tga");
//	}
//
//	delete model;
//	delete[] zbuffer;
//	delete[] shadowbuffer;
//	return 0;
//}

#include <vector>
#include <cstdlib>
#include <limits>
#include <iostream>
#include <algorithm>
#include "tgaimage.h"
#include "model.h"
#include "geometry.h"
#include "our_gl.h"

Model *model = NULL;

const int width = 256;
const int height = 256;

Vec3f       eye(1.2, -.8, 3);
Vec3f    center(0, 0, 0);
Vec3f        up(0, 1, 0);

struct ZShader : public IShader {
	mat<4, 3, float> varying_tri;

	ZShader() : varying_tri() {}

	virtual Vec4f vertex(int iface, int nthvert) {

		Vec4f gl_Vertex = embed<4>(model->vert(iface, nthvert)); // read the vertex from .obj file
		gl_Vertex = Projection*ModelView*gl_Vertex;          // transform it to screen coordinates
		varying_tri.set_col(nthvert, gl_Vertex);
		return gl_Vertex;
	}

	virtual bool fragment(Vec3f bar, TGAColor &color) {
		color = TGAColor(0, 0, 0);
		return false;
	}

	//virtual bool fragment(Vec3f gl_FragCoord, Vec3f bar, TGAColor &color) {
	//	color = TGAColor(0, 0, 0);
	//	return false;
	//}
};

float max_elevation_angle(float *zbuffer, Vec2f p, Vec2f dir) {
	float maxangle = 0;
	for (float t = 0.; t < 1000.; t += 1.) {
		Vec2f cur = p + dir * t;
		if (cur.x >= width || cur.y >= height || cur.x < 0 || cur.y < 0) return maxangle;

		float distance = (p - cur).norm();
		if (distance < 1.f) continue;
		float elevation = zbuffer[int(cur.x) + int(cur.y)*width] - zbuffer[int(p.x) + int(p.y)*width];
		maxangle = std::max(maxangle, atanf(elevation / distance));
	}
	return maxangle;
}

int main(int argc, char** argv) {
	if (2 == argc) {
		model = new Model(argv[1]);
    } else {
        model = new Model("obj/diablo3_pose/diablo3_pose.obj");
    }

	float *zbuffer = new float[width*height];
	for (int i = width * height; --i; ) {
		zbuffer[i] = -std::numeric_limits<float>::max();
	}

	TGAImage frame(width, height, TGAImage::RGB);
	lookat(eye, center, up);
	viewport(width / 8, height / 8, width * 3 / 4, height * 3 / 4);
	projection(-1.f / (eye - center).norm());

	ZShader zshader;
	Vec4f screen_coords[3];

	for (int i = 0; i < model->nfaces(); i++) {
		for (int j = 0; j < 3; j++) {
			screen_coords[j] = zshader.vertex(i, j);
		}
		triangle(zshader.varying_tri, zshader, frame, zbuffer);
	}

	for (int x = 0; x < width; x++) {
		for (int y = 0; y < height; y++) {
			if (zbuffer[x + y * width] < -1e5) continue;
			float total = 0;
			for (float a = 0; a < M_PI * 2 - 1e-4; a += M_PI / 4) {
				total += M_PI / 2 - max_elevation_angle(zbuffer, Vec2f(x, y), Vec2f(cos(a), sin(a)));
			}
			total /= (M_PI / 2) * 8;
			total = pow(total, 100.f);
			frame.set(x, y, TGAColor(total * 255, total * 255, total * 255));
		}
	}

	frame.flip_vertically();
	frame.write_tga_file("framebuffer.tga");
	delete[] zbuffer;
	delete model;
	return 0;
}