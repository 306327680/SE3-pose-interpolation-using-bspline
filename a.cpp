#include <iostream>
#include"cv.h"
#include"cxcore.h"
#include"highgui.h"
#include<cstdio>
#include<cmath>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include "sophus/so3.h"
#include "sophus/se3.h"

#include <pcl/common/transforms.h>
using std::cout;
using std::endl;

class SplineFusion{
public:
	SplineFusion() = default;
	//??23???
	Eigen::Isometry3d cumulativeForm(Eigen::Isometry3d T_1,Eigen::Isometry3d T_2,
									 Eigen::Isometry3d T_3,Eigen::Isometry3d T_4, double u);
	double getUt(double t, double ti, double dt){return (t-ti)/dt;};
	Sophus::SE3 fromAtoB(Sophus::SE3 a,Sophus::SE3 b);
	void SE3Eigen2Sophus(Eigen::Isometry3d e,Sophus::SE3 & s);
	void test();
private:
	Sophus::SE3 t1,t2,t3,t4;
	pcl::PointCloud<pcl::PointXYZI> after,pose,temp,out,cp_in,cp_out,cp_save;
protected:

};


Eigen::Isometry3d SplineFusion::cumulativeForm(Eigen::Isometry3d T_1,Eigen::Isometry3d T_2,
											   Eigen::Isometry3d T_3,Eigen::Isometry3d T_4, double u) {
	Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
	Sophus::SE3 cur;
	SE3Eigen2Sophus(T_1,t1);
	SE3Eigen2Sophus(T_2,t2);
	SE3Eigen2Sophus(T_3,t3);
	SE3Eigen2Sophus(T_4,t4);
	cur =   Sophus::SE3::exp(t1.log())*
			Sophus::SE3::exp(((5 + 3*u - 3*u*u + u*u*u) / 6)*fromAtoB(t1,t2).log())*
			Sophus::SE3::exp(((1 + 3*u + 3*u*u - 2*u*u*u) / 6)*fromAtoB(t2,t3).log())*
		    Sophus::SE3::exp(((u*u*u)/6)*fromAtoB(t3,t4).log());
	result = cur.matrix();
	return result;
	
}

void SplineFusion::SE3Eigen2Sophus(Eigen::Isometry3d e, Sophus::SE3 & s) {
	Sophus::SE3 t1;
	Eigen::Matrix4d temp;
	temp = e.matrix();
	Eigen::Vector3d t(temp(0,3),temp(1,3),temp(2,3));
	t1 = Sophus::SE3(e.rotation(),t);
	s = t1;
}

Sophus::SE3 SplineFusion::fromAtoB(Sophus::SE3 a, Sophus::SE3 b) {
	return Sophus::SE3(a.inverse()*b);
}

void SplineFusion::test() {
	Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI/90, Eigen::Vector3d(0,0,1)).toRotationMatrix();
	Eigen::Vector3d t(1,0,0);
	Eigen::Isometry3d temp_pose1;
	Eigen::Isometry3d temp_pose2;
	Eigen::Isometry3d temp_pose3;
	Eigen::Isometry3d temp_pose4;
	Eigen::Isometry3d temp_pose5;
	Eigen::Isometry3d temp_pose6;
	Eigen::Isometry3d temp_pose7;
	pcl::PointXYZI temp1,temp2,temp3;
	temp_pose1.setIdentity();
	temp_pose1.rotate(Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d(0,0,1)).toRotationMatrix());
	temp_pose1.translate(Eigen::Vector3d(0,0,0.5));
	cout<<temp_pose1.matrix()<<endl;
	temp_pose2.setIdentity();
	temp_pose2.rotate(Eigen::AngleAxisd(1.5*M_PI/4, Eigen::Vector3d(0,.1,1)).toRotationMatrix());
	temp_pose2.translate(Eigen::Vector3d(0,0,1));
	cout<<temp_pose2.matrix()<<endl;
	temp_pose3.setIdentity();
	temp_pose3.rotate(Eigen::AngleAxisd(2.2*M_PI/4, Eigen::Vector3d(0,.3,1)).toRotationMatrix());
	temp_pose3.translate(Eigen::Vector3d(0,0,2.3));
	cout<<temp_pose3.matrix()<<endl;
	temp_pose4.setIdentity();
	temp_pose4.rotate(Eigen::AngleAxisd(3.2*M_PI/4, Eigen::Vector3d(.5,0,1)).toRotationMatrix());
	temp_pose4.translate(Eigen::Vector3d(0,0,3.7));
	cout<<temp_pose4.matrix()<<endl;
	temp_pose5.setIdentity();
	temp_pose5.rotate(Eigen::AngleAxisd(2.1*M_PI/4, Eigen::Vector3d(.2,0,1)).toRotationMatrix());
	temp_pose5.translate(Eigen::Vector3d(0,1,3.7));
	temp_pose6.setIdentity();
	temp_pose6.rotate(Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d(0,-0.2,1)).toRotationMatrix());
	temp_pose6.translate(Eigen::Vector3d(1,1,3.7));
	temp_pose7.setIdentity();
	temp_pose7.rotate(Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d(0.4,0,1)).toRotationMatrix());
	temp_pose7.translate(Eigen::Vector3d(1,1.5,3.7));
	Sophus::SE3 a(Eigen::AngleAxisd(0, Eigen::Vector3d(0,0,1)).toRotationMatrix(),Eigen::Vector3d(0,0,0)),
			b(Eigen::AngleAxisd(M_PI, Eigen::Vector3d(0,0,1)).toRotationMatrix(),Eigen::Vector3d(1,2,1));
	cout<<"curr: a->b:"<<endl<<fromAtoB(a,b)<<endl;
	int intens = 0;
	temp2.y = 0;
	temp2.z = 0;
	for (double j = 0; j < 100; ++j) {
		temp2.x = j/100.0;
		temp2.y = 0;
		temp2.z = 0;
		temp2.intensity = 1;
		temp.push_back(temp2);
	}
	for (double j = 0; j < 100; ++j) {
		temp2.x = 0;
		temp2.y = j/100.0;
		temp2.intensity = 2;
		temp.push_back(temp2);
	}
	for (double j = 0; j < 100; ++j) {
		temp2.x = 0;
		temp2.y = 0;
		temp2.z = j/100.0;
		temp2.intensity = 3;
		temp.push_back(temp2);
	}
	cp_in = temp;
	pcl::transformPointCloud(cp_in,cp_out,temp_pose1.matrix());
	for (double j = 0; j < 300; ++j) {
		cp_save.push_back(cp_out[j]);
	}
	pcl::transformPointCloud(cp_in,cp_out,temp_pose2.matrix());
	for (double j = 0; j < 300; ++j) {
		cp_save.push_back(cp_out[j]);
	}
	pcl::transformPointCloud(cp_in,cp_out,temp_pose3.matrix());
	for (double j = 0; j < 300; ++j) {
		cp_save.push_back(cp_out[j]);
	}
	pcl::transformPointCloud(cp_in,cp_out,temp_pose4.matrix());
	for (double j = 0; j < 300; ++j) {
		cp_save.push_back(cp_out[j]);
	}
	pcl::transformPointCloud(cp_in,cp_out,temp_pose5.matrix());
	for (double j = 0; j < 300; ++j) {
		cp_save.push_back(cp_out[j]);
	}
	pcl::transformPointCloud(cp_in,cp_out,temp_pose6.matrix());
	for (double j = 0; j < 300; ++j) {
		cp_save.push_back(cp_out[j]);
	}
	pcl::transformPointCloud(cp_in,cp_out,temp_pose7.matrix());
	for (double j = 0; j < 300; ++j) {
		cp_save.push_back(cp_out[j]);
	}
	pcl::io::savePCDFile("cp_save.pcd",cp_save);
	
	
	
	for (double i = 0; i < 100; ++i) {
		Eigen::Isometry3d tf;
		Eigen::Matrix4d mat;
		tf = cumulativeForm(temp_pose1,temp_pose2,temp_pose3,temp_pose4,i/100.0);
		cout<<i<<endl<<tf.matrix()<<endl;
		mat = tf.matrix();
		temp1.x = mat(0,3);
		temp1.y = mat(1,3);
		temp1.z = mat(2,3);
		temp1.intensity = intens;
		intens++;
		pcl::transformPointCloud(temp,out,mat);
		for (double j = 0; j < 300; ++j) {
			pose.push_back(out[j]);
		}
		
		after.push_back(temp1);
	}
	for (double i = 0; i < 100; ++i) {
		Eigen::Isometry3d tf;
		Eigen::Matrix4d mat;
		tf = cumulativeForm(temp_pose2,temp_pose3,temp_pose4,temp_pose5,i/100.0);
		cout<<i<<endl<<tf.matrix()<<endl;
		mat = tf.matrix();
		temp1.x = mat(0,3);
		temp1.y = mat(1,3);
		temp1.z = mat(2,3);
		temp1.intensity = intens;
		intens++;
		pcl::transformPointCloud(temp,out,mat);
		for (double j = 0; j < 300; ++j) {
			pose.push_back(out[j]);
		}
		after.push_back(temp1);
	}
	for (double i = 0; i < 100; ++i) {
		Eigen::Isometry3d tf;
		Eigen::Matrix4d mat;
		tf = cumulativeForm(temp_pose3,temp_pose4,temp_pose5,temp_pose6,i/100.0);
		cout<<i<<endl<<tf.matrix()<<endl;
		mat = tf.matrix();
		temp1.x = mat(0,3);
		temp1.y = mat(1,3);
		temp1.z = mat(2,3);
		temp1.intensity = intens;
		intens++;
		pcl::transformPointCloud(temp,out,mat);
		for (double j = 0; j < 300; ++j) {
			pose.push_back(out[j]);
		}
		after.push_back(temp1);
	}
	for (double i = 0; i < 100; ++i) {
		Eigen::Isometry3d tf;
		Eigen::Matrix4d mat;
		tf = cumulativeForm(temp_pose4,temp_pose5,temp_pose6,temp_pose7,i/100.0);
		cout<<i<<endl<<tf.matrix()<<endl;
		mat = tf.matrix();
		temp1.x = mat(0,3);
		temp1.y = mat(1,3);
		temp1.z = mat(2,3);
		pcl::transformPointCloud(temp,out,mat);
		for (double j = 0; j < 300; ++j) {
			pose.push_back(out[j]);
		}
		temp1.intensity = intens;
		intens++;
		after.push_back(temp1);
	}

	pcl::io::savePCDFile("temp1.pcd",after);
	pcl::io::savePCDFile("pose.pcd",pose);
}


using namespace std;

int main(int argc, char *argv[])
{
	SplineFusion sf;
	sf.test();
	return 0;
}
