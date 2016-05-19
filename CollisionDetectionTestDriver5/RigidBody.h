#pragma once

#include <d3dx9.h>
#include <vector>
#include <assert.h>

struct RigidBody
{
	D3DXVECTOR3 position; //�ʒu
	D3DXQUATERNION orientation; //�p��

	D3DXVECTOR3 linear_velocity; //���i���x
	D3DXVECTOR3 angular_velocity; //�p���x

	//��������(inertial_mass)�̒�`�˃R���X�g���N�^�œK�؂ȏ����l��^���邱��
	FLOAT inertial_mass;
	//�͂̃A�L�������[�^(accumulated_force)�̒�`�˃R���X�g���N�^�œK�؂ȏ����l��^���邱��
	D3DXVECTOR3 accumulated_force;

	//�������[�����g(inertia_tensor)�̒�`�˃R���X�g���N�^�œK�؂ȏ����l��^���邱��
	D3DXMATRIX inertia_tensor;
	//�g���N�A�L�������[�^(accumulated_torque)�̒�`�˃R���X�g���N�^�œK�؂ȏ����l��^���邱��
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
		//���I�u�W�F�N�g�̏ꍇ�̂݃C���e�O���[�V�������s��
		if(is_movable()) 
		{
			//��(accumulated_force)��������x(linear_acceleration)���Z�o�����x(linear_velocity)���X�V����
			D3DXVECTOR3 linear_acceleration;
			linear_acceleration = accumulated_force / inertial_mass;
			linear_velocity += linear_acceleration * duration;

			//���i���x�ɂ��ʒu�̍X�V
			position += linear_velocity * duration;

			//�g���N(accumulated_torque)����p�����x(angular_acceleration)���Z�o���p���x(angular_velocity)���X�V����
			D3DXMATRIX inverse_inertia_tensor;
			D3DXMatrixInverse(&inverse_inertia_tensor, 0, &inertia_tensor);
			D3DXMATRIX rotation, transposed_rotation;
			D3DXMatrixRotationQuaternion(&rotation, &orientation);
			D3DXMatrixTranspose(&transposed_rotation, &rotation);
			inverse_inertia_tensor = transposed_rotation * inverse_inertia_tensor * rotation;
			D3DXVECTOR3 angular_acceleration;
			D3DXVec3TransformCoord(&angular_acceleration, &accumulated_torque, &inverse_inertia_tensor);
			angular_velocity += angular_acceleration * duration;

			//�p���x�ɂ��p���̍X�V
			D3DXQUATERNION w(angular_velocity.x, angular_velocity.y, angular_velocity.z, 0);
			w = orientation * w;
			orientation += 0.5f * w * duration;
			D3DXQuaternionNormalize(&orientation, &orientation);
		}
		//�͂̃A�L�������[�^���[�����Z�b�g����
		accumulated_force = D3DXVECTOR3(0, 0, 0);
		//�g���N�̃A�L�������[�^���[�����Z�b�g����
		accumulated_torque = D3DXVECTOR3(0, 0, 0);
	}

	void add_force(const D3DXVECTOR3 &force)
	{
		//�A�L�������[�^(accumulated_force)�ɗ�(force)��ݎZ����
		accumulated_force += force;
	}
	void add_torque(const D3DXVECTOR3 &torque)
	{
		//�A�L�������[�^(accumulated_torque)�Ƀg���N(torque)��ݎZ����
		accumulated_torque += torque;
	}
	void add_force_at_point(const D3DXVECTOR3 &force, const D3DXVECTOR3 &point/*���[���h���W*/)
	{
		//�g���N(���[�����g)���v�Z����
		D3DXVECTOR3 torque;
		D3DXVec3Cross(&torque, &(point - position), &force);
		//�A�L�������[�^(accumulated_force)�ɗ�(force)��ݎZ����
		accumulated_force += force;
		//�A�L�������[�^(accumulated_torque)�Ƀg���N(torque)��ݎZ����
		accumulated_torque += torque;
	}

	//�T�C�Y�擾�֐�(�������z�֐�)
	virtual D3DXVECTOR3 get_dimension() const = 0;

	//���I�u�W�F�N�g���H
	bool is_movable() const
	{
		//��������(inertial_mass)��0���傫��FLT_MAX��菬�����ꍇ�͉��I�u�W�F�N�g�Ƃ݂Ȃ�
		return (inertial_mass > 0 && inertial_mass < FLT_MAX);
	}

	//��������(inertial_mass)�̋t����Ԃ��A�s���I�u�W�F�N�g�ꍇ��0��Ԃ�
	FLOAT inverse_mass() const
	{
		return (inertial_mass > 0 && inertial_mass < FLT_MAX) ? 1.0f / inertial_mass : 0;
	}

	//�������[�����g�e���\��(inertia_tensor)�̋t�s���Ԃ�
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

//���̃N���X�̒�`�E����
struct Sphere : public RigidBody
{
	FLOAT r;	//���a
	Sphere(FLOAT r/*���a*/, FLOAT density/*���x*/) : r(r)
	{
		//��������(inertial_mass)���v�Z
		inertial_mass = 4.0f * 3.14159265358979f * r * r * r / 3.0f * density;

		//�������[�����g(inertia_tensor)���v�Z
		D3DXMatrixIdentity(&inertia_tensor);
		inertia_tensor._11 = 0.4f * inertial_mass * r * r;
		inertia_tensor._22 = 0.4f * inertial_mass * r * r;
		inertia_tensor._33 = 0.4f * inertial_mass * r * r;
	}

	//�T�C�Y(dimension)�̎擾�֐��̎���(�I�[�o�[���C�h)
	virtual D3DXVECTOR3 get_dimension() const
	{
		return D3DXVECTOR3(r, r, r);
	}
};

//�{�b�N�X�N���X�̒�`�E����
struct Box : public RigidBody
{
	D3DXVECTOR3 half_size;	//���Ӓ�
	Box(D3DXVECTOR3 half_size/*���Ӓ�*/, FLOAT density/*���x*/) : half_size(half_size)
	{
		//��������(inertial_mass)���v�Z
		inertial_mass = (half_size.x * half_size.y * half_size.z) * 8.0f * density;

		//�������[�����g(inertia_tensor)���v�Z
		D3DXMatrixIdentity(&inertia_tensor);
		inertia_tensor._11 = 0.3333333f * inertial_mass * ((half_size.y * half_size.y) + (half_size.z * half_size.z));
		inertia_tensor._22 = 0.3333333f * inertial_mass * ((half_size.z * half_size.z) + (half_size.x * half_size.x));
		inertia_tensor._33 = 0.3333333f * inertial_mass * ((half_size.x * half_size.x) + (half_size.y * half_size.y));
	}

	//�T�C�Y�擾�֐��̎���(�I�[�o�[���C�h)
	virtual D3DXVECTOR3 get_dimension() const
	{
		return half_size;
	}
};

//���ʃN���X�̒�`�E����
struct Plane : public RigidBody
{
	//�s���I�u�W�F�N�g�Ƃ��Đ�������
	Plane(D3DXVECTOR3 n, FLOAT d)
	{
		//��������(inertial_mass)��FLT_MAX���Z�b�g
		inertial_mass = FLT_MAX;

		//�������[�����g(inertia_tensor)�̑Ίp������FLT_MAX���Z�b�g
		D3DXMatrixIdentity(&inertia_tensor);
		inertia_tensor._11 = FLT_MAX;
		inertia_tensor._22 = FLT_MAX;
		inertia_tensor._33 = FLT_MAX;

		//n = (0, 1, 0),d = 0����{�ʒu�E�p���Ƃ��āA������n,d���猻�݂̈ʒu(position)�Ǝp��(orientation)���v�Z����
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

	//�T�C�Y�擾�֐��̎���(�I�[�o�[���C�h)
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

	D3DXVECTOR3 point;	//�ڐG�_
	D3DXVECTOR3 normal; //����0(body[0])���猩���ڐG�ʂ̖@��
	FLOAT penetration;	//�߂荞�ݗ�
	FLOAT restitution;	//�����W��

	void resolve();	//�ڐG�̉���

};	

INT generate_contact_sphere_sphere(Sphere *s0, Sphere *s1, std::vector<Contact> *contacts, FLOAT restitution);
INT generate_contact_sphere_plane(Sphere *sphere, Plane *plane, std::vector<Contact> *contacts, FLOAT restitution, BOOL half_space = TRUE);
INT generate_contact_sphere_box(Sphere *sphere, Box *box, std::vector<Contact> *contacts, FLOAT restitution);
INT generate_contact_box_plane(Box *box, Plane *plane, std::vector<Contact> *contacts, FLOAT restitution);
INT generate_contact_box_box(Box *b0, Box *b1, std::vector<Contact> *contacts, FLOAT restitution);
