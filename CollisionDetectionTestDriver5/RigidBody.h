#pragma once

#include <d3dx9.h>
#include <vector>
#include <assert.h>

struct RigidBody
{
	D3DXVECTOR3 position; //位置
	D3DXQUATERNION orientation; //姿勢

	D3DXVECTOR3 linear_velocity; //並進速度
	D3DXVECTOR3 angular_velocity; //角速度

	//慣性質量(inertial_mass)の定義⇒コンストラクタで適切な初期値を与えること
	FLOAT inertial_mass;
	//力のアキュムレータ(accumulated_force)の定義⇒コンストラクタで適切な初期値を与えること
	D3DXVECTOR3 accumulated_force;

	//慣性モーメント(inertia_tensor)の定義⇒コンストラクタで適切な初期値を与えること
	D3DXMATRIX inertia_tensor;
	//トルクアキュムレータ(accumulated_torque)の定義⇒コンストラクタで適切な初期値を与えること
	D3DXVECTOR3 accumulated_torque;

	RigidBody() :
		position(0, 0, 0), orientation(0, 0, 0, 1),
		linear_velocity(0, 0, 0), angular_velocity(0, 0, 0),
		inertial_mass(1), accumulated_force(0, 0, 0),
		accumulated_torque(0, 0, 0)
	{
		D3DXMatrixIdentity(&inertia_tensor);
	}

	void integrate(FLOAT duration)
	{
		//可動オブジェクトの場合のみインテグレーションを行う
		if(is_movable()) 
		{
			//力(accumulated_force)から加速度(linear_acceleration)を算出し速度(linear_velocity)を更新する
			D3DXVECTOR3 linear_acceleration;
			linear_acceleration = accumulated_force / inertial_mass;
			linear_velocity += linear_acceleration * duration;

			//並進速度による位置の更新
			position += linear_velocity * duration;

			//トルク(accumulated_torque)から角加速度(angular_acceleration)を算出し角速度(angular_velocity)を更新する
			D3DXMATRIX inverse_inertia_tensor;
			D3DXMatrixInverse(&inverse_inertia_tensor, 0, &inertia_tensor);
			D3DXMATRIX rotation, transposed_rotation;
			D3DXMatrixRotationQuaternion(&rotation, &orientation);
			D3DXMatrixTranspose(&transposed_rotation, &rotation);
			inverse_inertia_tensor = transposed_rotation * inverse_inertia_tensor * rotation;
			D3DXVECTOR3 angular_acceleration;
			D3DXVec3TransformCoord(&angular_acceleration, &accumulated_torque, &inverse_inertia_tensor);
			angular_velocity += angular_acceleration * duration;

			//角速度による姿勢の更新
			D3DXQUATERNION w(angular_velocity.x, angular_velocity.y, angular_velocity.z, 0);
			w = orientation * w;
			orientation += 0.5f * w * duration;
			D3DXQuaternionNormalize(&orientation, &orientation);
		}
		//力のアキュムレータをゼロリセットする
		accumulated_force = D3DXVECTOR3(0, 0, 0);
		//トルクのアキュムレータをゼロリセットする
		accumulated_torque = D3DXVECTOR3(0, 0, 0);
	}

	void add_force(const D3DXVECTOR3 &force)
	{
		//アキュムレータ(accumulated_force)に力(force)を累算する
		accumulated_force += force;
	}
	void add_torque(const D3DXVECTOR3 &torque)
	{
		//アキュムレータ(accumulated_torque)にトルク(torque)を累算する
		accumulated_torque += torque;
	}
	void add_force_at_point(const D3DXVECTOR3 &force, const D3DXVECTOR3 &point/*ワールド座標*/)
	{
		//トルク(モーメント)を計算する
		D3DXVECTOR3 torque;
		D3DXVec3Cross(&torque, &(point - position), &force);
		//アキュムレータ(accumulated_force)に力(force)を累算する
		accumulated_force += force;
		//アキュムレータ(accumulated_torque)にトルク(torque)を累算する
		accumulated_torque += torque;
	}

	//サイズ取得関数(純粋仮想関数)
	virtual D3DXVECTOR3 get_dimension() const = 0;

	//可動オブジェクトか？
	bool is_movable() const
	{
		//慣性質量(inertial_mass)が0より大きくFLT_MAXより小さい場合は可動オブジェクトとみなす
		return (inertial_mass > 0 && inertial_mass < FLT_MAX);
	}

	//慣性質量(inertial_mass)の逆数を返し、不可動オブジェクト場合は0を返す
	FLOAT inverse_mass() const
	{
		return (inertial_mass > 0 && inertial_mass < FLT_MAX) ? 1.0f / inertial_mass : 0;
	}

	//慣性モーメントテンソル(inertia_tensor)の逆行列を返す
	D3DXMATRIX inverse_inertia_tensor(bool transformed = true) const
	{
		D3DXMATRIX inverse_inertia_tensor;
		if (is_movable())
		{
			D3DXMatrixInverse(&inverse_inertia_tensor, 0, &inertia_tensor);
			if (transformed)
			{
				D3DXMATRIX rotation, transposed_rotation;
				D3DXMatrixRotationQuaternion(&rotation, &orientation);
				D3DXMatrixTranspose(&transposed_rotation, &rotation);
				inverse_inertia_tensor = transposed_rotation * inverse_inertia_tensor * rotation;
			}
		}
		else
		{
			D3DXMatrixIdentity(&inverse_inertia_tensor);
			inverse_inertia_tensor._11 = FLT_EPSILON;
			inverse_inertia_tensor._22 = FLT_EPSILON;
			inverse_inertia_tensor._33 = FLT_EPSILON;
		}
		return inverse_inertia_tensor;
	}
};

//球体クラスの定義・実装
struct Sphere : public RigidBody
{
	FLOAT r;	//半径
	Sphere(FLOAT r/*半径*/, FLOAT density/*密度*/) : r(r)
	{
		//慣性質量(inertial_mass)を計算
		inertial_mass = 4.0f * 3.14159265358979f * r * r * r / 3.0f * density;

		//慣性モーメント(inertia_tensor)を計算
		D3DXMatrixIdentity(&inertia_tensor);
		inertia_tensor._11 = 0.4f * inertial_mass * r * r;
		inertia_tensor._22 = 0.4f * inertial_mass * r * r;
		inertia_tensor._33 = 0.4f * inertial_mass * r * r;
	}

	//サイズ(dimension)の取得関数の実装(オーバーライド)
	virtual D3DXVECTOR3 get_dimension() const
	{
		return D3DXVECTOR3(r, r, r);
	}
};

//ボックスクラスの定義・実装
struct Box : public RigidBody
{
	D3DXVECTOR3 half_size;	//半辺長
	Box(D3DXVECTOR3 half_size/*半辺長*/, FLOAT density/*密度*/) : half_size(half_size)
	{
		//慣性質量(inertial_mass)を計算
		inertial_mass = (half_size.x * half_size.y * half_size.z) * 8.0f * density;

		//慣性モーメント(inertia_tensor)を計算
		D3DXMatrixIdentity(&inertia_tensor);
		inertia_tensor._11 = 0.3333333f * inertial_mass * ((half_size.y * half_size.y) + (half_size.z * half_size.z));
		inertia_tensor._22 = 0.3333333f * inertial_mass * ((half_size.z * half_size.z) + (half_size.x * half_size.x));
		inertia_tensor._33 = 0.3333333f * inertial_mass * ((half_size.x * half_size.x) + (half_size.y * half_size.y));
	}

	//サイズ取得関数の実装(オーバーライド)
	virtual D3DXVECTOR3 get_dimension() const
	{
		return half_size;
	}
};

//平面クラスの定義・実装
struct Plane : public RigidBody
{
	//不動オブジェクトとして生成する
	Plane(D3DXVECTOR3 n, FLOAT d)
	{
		//慣性質量(inertial_mass)にFLT_MAXをセット
		inertial_mass = FLT_MAX;

		//慣性モーメント(inertia_tensor)の対角成分にFLT_MAXをセット
		D3DXMatrixIdentity(&inertia_tensor);
		inertia_tensor._11 = FLT_MAX;
		inertia_tensor._22 = FLT_MAX;
		inertia_tensor._33 = FLT_MAX;

		//n = (0, 1, 0),d = 0を基本位置・姿勢として、引数のn,dから現在の位置(position)と姿勢(orientation)を計算する
		D3DXVec3Normalize(&n, &n);
		position = d * n;

		D3DXVECTOR3 Y(0, 1, 0);
		FLOAT angle = acosf(D3DXVec3Dot(&Y, &n));
		D3DXVECTOR3 axis;
		D3DXVec3Cross(&axis, &Y, &n);
		D3DXVec3Normalize(&axis, &axis);
		D3DXQuaternionRotationAxis(&orientation, &axis, angle);
		//D3DXQuaternionNormalize(&orientation, &orientation);
	}

	//サイズ取得関数の実装(オーバーライド)
	virtual D3DXVECTOR3 get_dimension() const
	{
		return D3DXVECTOR3(1, 0, 1);
	}
};

struct Contact
{
	Contact() : point(0, 0, 0), normal(0, 0, 0), penetration(0), restitution(0)
	{
		body[0] = body[1] = 0;
	}

	RigidBody* body[2];

	D3DXVECTOR3 point;	//接触点
	D3DXVECTOR3 normal; //剛体0(body[0])から見た接触面の法線
	FLOAT penetration;	//めり込み量
	FLOAT restitution;	//反発係数

	void resolve();	//接触の解消

};	

INT generate_contact_sphere_sphere(Sphere *s0, Sphere *s1, std::vector<Contact> *contacts, FLOAT restitution);
INT generate_contact_sphere_plane(Sphere *sphere, Plane *plane, std::vector<Contact> *contacts, FLOAT restitution, BOOL half_space = TRUE);
INT generate_contact_sphere_box(Sphere *sphere, Box *box, std::vector<Contact> *contacts, FLOAT restitution);
INT generate_contact_box_plane(Box *box, Plane *plane, std::vector<Contact> *contacts, FLOAT restitution);
INT generate_contact_box_box(Box *b0, Box *b1, std::vector<Contact> *contacts, FLOAT restitution);
