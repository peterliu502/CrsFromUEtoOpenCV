#include "CrsFromUEtoOpenCV.h"
using namespace std;
using namespace cv;

struct Transformer
{
	/* Rotation of this transformation, as a quaternion. */
	Quat<float> Rotation;

	/* Translation of this transformation, as a vector. */
	Vec3f Translation;

	/* 3D scale (always applied in local space) as a vector. */
	Vec3f Scale3D = (1, 1, 1);

	Mat toMat() {
		Mat rst = Mat::eye(4, 4, CV_32FC1);

		// Insert translation vector
		rst.ptr<float>(0)[3] = Translation[0];
		rst.ptr<float>(1)[3] = Translation[1];
		rst.ptr<float>(2)[3] = Translation[2];

		// Insert rotation matrix
		Matx rMatx = Rotation.normalize().toRotMat3x3();
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				rst.ptr<float>(i)[j] = rMatx(i, j);
			}
		}

		return rst;
	}

	void SetFromMatrix(const Mat& InMatrix)
	{
		Mat M44 = InMatrix;

		// Extract rotation matrix
		Mat M33 = Mat::eye(3, 3, CV_32FC1);
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				M33.ptr<float>(i)[j] = M44.ptr<float>(i)[j];
			}
		}
		Rotation = Quat<float>::createFromRotMat(M33);
		Rotation.normalize();

		// Extract translation vector
		Translation = Vec3f(M44.ptr<float>(0)[3], M44.ptr<float>(1)[3], M44.ptr<float>(2)[3]);
	}

private:
	enum EAxis { X = 0, Y = 1, Z = 2, Xn = 3, Yn = 4, Zn = 5 };

	const vector<vector<float>> UnitVectors =
	{
		{  1,  0,  0 }, //  X
		{  0,  1,  0 }, //  Y
		{  0,  0,  1 }, //  Z
		{ -1,  0,  0 }, // -X
		{  0, -1,  0 }, // -Y
		{  0,  0, -1 }, // -Z
	};

	Mat UnitVectorFromAxisEnum(EAxis Axis)
	{
		Mat rst = Mat::zeros(3, 1, CV_32FC1);
		auto uv = UnitVectors[std::underlying_type_t<EAxis>(Axis)];
		rst.ptr<float>(0)[0] = uv[0];
		rst.ptr<float>(1)[0] = uv[1];
		rst.ptr<float>(2)[0] = uv[2];
		return rst;
	};

	void ConvertCoordinateSystem(const EAxis SrcXInDstAxis, const EAxis SrcYInDstAxis, const EAxis SrcZInDstAxis)
	{
		// Unreal Engine:
		//   Front : X
		//   Right : Y
		//   Up    : Z
		//
		// OpenCV:
		//   Front : Z
		//   Right : X
		//   Up    : Yn

		Mat M33 = Mat::eye(3, 3, CV_32FC1);
		Mat M44 = Mat::eye(4, 4, CV_32FC1);
		UnitVectorFromAxisEnum(SrcXInDstAxis).col(0).copyTo(M33.col(0));
		UnitVectorFromAxisEnum(SrcYInDstAxis).col(0).copyTo(M33.col(1));
		UnitVectorFromAxisEnum(SrcZInDstAxis).col(0).copyTo(M33.col(2));
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				M44.ptr<float>(i)[j] = M33.ptr<float>(i)[j];
			}
		}

		SetFromMatrix(M44.t() * toMat() * M44);
	}

public:
	/* Convert from UE to OpenCV. */
	void convertUnrealToOpenCV()
	{
		ConvertCoordinateSystem(EAxis::Y, EAxis::Zn, EAxis::X);
	}

	/* Convert OpenCV to UE. */
	void convertOpenCVToUnreal()
	{
		ConvertCoordinateSystem(EAxis::Z, EAxis::X, EAxis::Yn);
	}
};

int main()
{
	// Test for translation
	vector<Vec3f> vv3f1;
	vv3f1.push_back(Vec3f(0, 0, 0));
	vv3f1.push_back(Vec3f(1, 0, 0));
	vv3f1.push_back(Vec3f(1, 1, 0));
	vv3f1.push_back(Vec3f(0, 1, 0));
	vv3f1.push_back(Vec3f(0, 0, 1));
	vv3f1.push_back(Vec3f(1, 0, 1));
	vv3f1.push_back(Vec3f(1, 1, 1));
	vv3f1.push_back(Vec3f(0, 1, 1));

	for (auto v : vv3f1) {
		Transformer ts;
		Quat<float> q = Quat<float>::createFromEulerAngles(Vec3f(0,0,0), QuatEnum::EulerAnglesType::EXT_XYZ);
		ts.Rotation = q;
		ts.Translation = v;
		ts.convertOpenCVToUnreal();
		cout << ts.Translation << endl;
	}

	// Test for rotation
	vector<Vec3f> vv3f2;
	vv3f2.push_back(Vec3f(1, 0, 0));
	vv3f2.push_back(Vec3f(0, 1, 0));
	vv3f2.push_back(Vec3f(0, 0, 1));

	for (auto v : vv3f2) {
		Transformer ts;
		Vec3f t = (0, 0, 0);
		Quat<float> q = Quat<float>::createFromEulerAngles(v, QuatEnum::EulerAnglesType::EXT_XYZ);
		ts.Rotation = q;
		ts.Translation = t;
		ts.convertOpenCVToUnreal();
		cout << ts.Rotation.toEulerAngles(QuatEnum::EulerAnglesType::EXT_XYZ) << endl;
	}

	return 0;
}
