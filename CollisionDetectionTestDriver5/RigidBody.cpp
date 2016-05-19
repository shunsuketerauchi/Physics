#define NOMINMAX
#include <assert.h>
#include "RigidBody.h"
#include "DebugDrawManager.h"

inline void rotate_vector_by_quaternion(D3DXVECTOR3 *out, const D3DXQUATERNION &q, const D3DXVECTOR3 &v)
{
#if 1
	D3DXVECTOR3 p;
	p.x = 2.0f * (q.x * v.x + q.y * v.y + q.z * v.z) * q.x + (q.w * q.w - (q.x * q.x + q.y * q.y + q.z * q.z)) * v.x + 2.0f * q.w * (q.y * v.z - q.z * v.y);
	p.y = 2.0f * (q.x * v.x + q.y * v.y + q.z * v.z) * q.y + (q.w * q.w - (q.x * q.x + q.y * q.y + q.z * q.z)) * v.y + 2.0f * q.w * (q.z * v.x - q.x * v.z);
	p.z = 2.0f * (q.x * v.x + q.y * v.y + q.z * v.z) * q.z + (q.w * q.w - (q.x * q.x + q.y * q.y + q.z * q.z)) * v.z + 2.0f * q.w * (q.x * v.y - q.y * v.x);
	*out = p;
#else
	D3DXQUATERNION conjugate, p( v.x, v.y, v.z, 0 );
	D3DXQuaternionConjugate( &conjugate, &q );
	p = conjugate * p * q;
	out->x = p.x;
	out->y = p.y;
	out->z = p.z;
#endif
}

INT generate_contact_sphere_sphere(Sphere *s0, Sphere *s1, std::vector<Contact> *contacts, FLOAT restitution)
{
	//2�̋��̂̏Փ˔�����s��
	//�Փ˂��Ă���ꍇ��Contact�I�u�W�F�N�g�𐶐�����
	//Contact�̑S�Ẵ����o�ϐ��ɒl���Z�b�g���A�R���e�i(contacts)�ɒǉ�����
	assert(s0 != s1);

	D3DXVECTOR3 p0 = s0->position;
	D3DXVECTOR3 p1 = s1->position;

	D3DXVECTOR3 n = p0 - p1;
	FLOAT l = D3DXVec3Length(&n);
	D3DXVec3Normalize(&n, &n);
	if (l < s0->r + s1->r)
	{
		Contact contact;
		contact.normal = n;
		contact.penetration = s0->r + s1->r - l;
		//contact.point = p1 + 0.5f * l * n;
		contact.point = p1 + (s1->r - (0.5f * contact.penetration)) * n;
		contact.body[0] = s0;
		contact.body[1] = s1;
		contact.restitution = restitution;
		contacts->push_back(contact);
		return 1;
	}
	return 0;
}
INT generate_contact_sphere_plane(Sphere *sphere, Plane *plane, std::vector<Contact> *contacts, FLOAT restitution, BOOL half_space)
{
	//���̂ƕ��ʂ̏Փ˔�����s��
	//�Փ˂��Ă���ꍇ��Contact�I�u�W�F�N�g�𐶐�����
	//Contact�̑S�Ẵ����o�ϐ��ɒl���Z�b�g���A�R���e�i(contacts)�ɒǉ�����
	//��half_space���^�̏ꍇ�͕ЖʁA�U�̏ꍇ�͗��ʂ̏Փ˔�����s��
	D3DXMATRIX matrix, inverse_matrix;
	D3DXMatrixRotationQuaternion(&matrix, &plane->orientation);
	matrix._41 = plane->position.x;
	matrix._42 = plane->position.y;
	matrix._43 = plane->position.z;
	D3DXVECTOR3 n(matrix._21, matrix._22, matrix._23);
	D3DXMatrixInverse(&inverse_matrix, 0, &matrix);

	D3DXVECTOR3 p;
	D3DXVec3TransformCoord(&p, &sphere->position, &inverse_matrix);

	//Half-space
	if (half_space && p.y < 0) return 0;

	if (fabsf(p.y) < sphere->r)
	{
		Contact contact;
		contact.normal = p.y > 0 ? n : -n;
		contact.point = sphere->position + p.y * -n;
		contact.penetration = sphere->r - fabsf(p.y);
		contact.body[0] = sphere;
		contact.body[1] = plane;
		contact.restitution = restitution;
		contacts->push_back(contact);
		return 1;
	}
	return 0;
}
INT generate_contact_sphere_box(Sphere *sphere, Box *box, std::vector<Contact> *contacts, FLOAT restitution)
{
	//���̂ƒ����̂̏Փ˔�����s��
	//�Փ˂��Ă���ꍇ��Contact�I�u�W�F�N�g�𐶐�����
	//Contact�̑S�Ẵ����o�ϐ��ɒl���Z�b�g���A�R���e�i(contacts)�ɒǉ�����
	D3DXMATRIX matrix, inverse_matrix;
	D3DXMatrixRotationQuaternion(&matrix, &box->orientation);
	matrix._41 = box->position.x;
	matrix._42 = box->position.y;
	matrix._43 = box->position.z;
	D3DXMatrixInverse(&inverse_matrix, 0, &matrix);

	D3DXVECTOR3 center;
	D3DXVec3TransformCoord(&center, &sphere->position, &inverse_matrix);

	//if (fabsf(center.x) - sphere->r > box->half_size.x ||
	//	fabsf(center.y) - sphere->r > box->half_size.y ||
	//	fabsf(center.z) - sphere->r > box->half_size.z)
	//{
	//	return 0;
	//}

	D3DXVECTOR3 closest_point;

	closest_point.x = center.x;
	if (center.x > box->half_size.x) closest_point.x = box->half_size.x;
	if (center.x < -box->half_size.x) closest_point.x = -box->half_size.x;

	closest_point.y = center.y;
	if (center.y > box->half_size.y) closest_point.y = box->half_size.y;
	if (center.y < -box->half_size.y) closest_point.y = -box->half_size.y;

	closest_point.z = center.z;
	if (center.z > box->half_size.z) closest_point.z = box->half_size.z;
	if (center.z < -box->half_size.z) closest_point.z = -box->half_size.z;

	FLOAT distance = D3DXVec3Length(&(closest_point - center));
	if (distance < sphere->r && distance > FLT_EPSILON)
	{
		D3DXVec3TransformCoord(&closest_point, &closest_point, &matrix);

		Contact contact;
		contact.normal = sphere->position - closest_point;
		contact.point = closest_point;
		contact.penetration = sphere->r - distance;
		contact.body[0] = sphere;
		contact.body[1] = box;
		contact.restitution = restitution;
		contacts->push_back(contact);

		return 1;
	}

	return 0;
}
INT generate_contact_box_plane(Box *box, Plane *plane, std::vector<Contact> *contacts, FLOAT restitution)
{
	//�����̂ƕ��ʂ̏Փ˔�����s��
	//�Փ˂��Ă���ꍇ��Contact�I�u�W�F�N�g�𐶐�����
	//Contact�̑S�Ẵ����o�ϐ��ɒl���Z�b�g���A�R���e�i(contacts)�ɒǉ�����
	D3DXVECTOR3 vertices[8] =
	{
		D3DXVECTOR3(-box->half_size.x, -box->half_size.y, -box->half_size.z),
		D3DXVECTOR3(-box->half_size.x, -box->half_size.y, +box->half_size.z),
		D3DXVECTOR3(-box->half_size.x, +box->half_size.y, -box->half_size.z),
		D3DXVECTOR3(-box->half_size.x, +box->half_size.y, +box->half_size.z),
		D3DXVECTOR3(+box->half_size.x, -box->half_size.y, -box->half_size.z),
		D3DXVECTOR3(+box->half_size.x, -box->half_size.y, +box->half_size.z),
		D3DXVECTOR3(+box->half_size.x, +box->half_size.y, -box->half_size.z),
		D3DXVECTOR3(+box->half_size.x, +box->half_size.y, +box->half_size.z)
	};

	D3DXVECTOR3 n;
	rotate_vector_by_quaternion(&n, plane->orientation, D3DXVECTOR3(0, 1, 0));
	FLOAT d = D3DXVec3Dot(&n, &plane->position);

	INT contacts_used = 0;
	for (int i = 0; i < 8; i++)
	{
		rotate_vector_by_quaternion(&vertices[i], box->orientation, vertices[i]);
		vertices[i] += box->position;


		FLOAT distance = D3DXVec3Dot(&vertices[i], &n);

		if (distance < d)
		{
			Contact contact;
			contact.normal = n;
			contact.point = vertices[i];
			contact.penetration = d - distance;
			contact.body[0] = box;
			contact.body[1] = plane;
			contact.restitution = restitution;
			contacts->push_back(contact);

			contacts_used++;
		}
	}
	return contacts_used;
}

enum SAT_TYPE
{
	POINTA_FACETB,
	POINTB_FACETA,
	EDGE_EDGE
};
struct OBB {
	D3DXVECTOR3 c; // OBB center point
	D3DXVECTOR3 u[3]; // Local x-, y-, and z-axes
	D3DXVECTOR3 e; // Positive halfwidth extents of OBB along each axis
};

static inline FLOAT sum_of_projected_radii(
	const OBB &obb,
	const D3DXVECTOR3 &axis
	)
{
	return
		fabsf(D3DXVec3Dot(&axis, &(obb.e.x * obb.u[0]))) +
		fabsf(D3DXVec3Dot(&axis, &(obb.e.y * obb.u[1]))) +
		fabsf(D3DXVec3Dot(&axis, &(obb.e.z * obb.u[2])));
}

//SAT (Separating Axis Theorem)
INT sat_obb_obb(const OBB &a, const OBB &b,
	FLOAT &smallest_penetration,
	INT smallest_axis[2],
	SAT_TYPE &smallest_case
	)
{
	smallest_penetration = FLT_MAX;
	FLOAT penetration = 0;

	FLOAT ra, rb;
	D3DXVECTOR3 L;
	D3DXVECTOR3 T = b.c - a.c;

	// Test axes L = A0, L = A1, L = A2
	for (int i = 0; i < 3; i++)
	{
		L = a.u[i];
		ra = a.e[i];
		rb = sum_of_projected_radii(b, L);
		penetration = ra + rb - fabsf(L.x * T.x + L.y * T.y + L.z * T.z);
		if (penetration < 0) return 0;
		if (smallest_penetration > penetration)
		{
			smallest_penetration = penetration;
			smallest_axis[0] = i;
			smallest_axis[1] = -1;
			smallest_case = POINTB_FACETA;
		}
	}

	// Test axes L = B0, L = B1, L = B2
	for (int i = 0; i < 3; i++)
	{
		L = b.u[i];
		ra = sum_of_projected_radii(a, L);
		rb = b.e[i];
		penetration = ra + rb - fabsf(L.x * T.x + L.y * T.y + L.z * T.z);
		if (penetration < 0) return 0;
		if (smallest_penetration > penetration)
		{
			smallest_penetration = penetration;
			smallest_axis[0] = -1;
			smallest_axis[1] = i;
			smallest_case = POINTA_FACETB;
		}
	}

	// Test axis L = A0 x B0
	D3DXVec3Cross(&L, &a.u[0], &b.u[0]);
	if (D3DXVec3LengthSq(&L) > FLT_EPSILON) //Is A0 not parallel to B0?
	{
		D3DXVec3Normalize(&L, &L);
		ra = sum_of_projected_radii(a, L);
		rb = sum_of_projected_radii(b, L);
		penetration = ra + rb - fabsf(L.x * T.x + L.y * T.y + L.z * T.z);
		if (penetration < 0) return 0;
		if (smallest_penetration > penetration)
		{
			smallest_penetration = penetration;
			smallest_axis[0] = 0;
			smallest_axis[1] = 0;
			smallest_case = EDGE_EDGE;
		}
	}

	// Test axis L = A0 x B1
	D3DXVec3Cross(&L, &a.u[0], &b.u[1]);
	if (D3DXVec3LengthSq(&L) > FLT_EPSILON)
	{
		D3DXVec3Normalize(&L, &L);
		ra = sum_of_projected_radii(a, L);
		rb = sum_of_projected_radii(b, L);
		penetration = ra + rb - fabsf(L.x * T.x + L.y * T.y + L.z * T.z);
		if (penetration < 0) return 0;
		if (smallest_penetration > penetration)
		{
			smallest_penetration = penetration;
			smallest_axis[0] = 0;
			smallest_axis[1] = 1;
			smallest_case = EDGE_EDGE;
		}
	}
	// Test axis L = A0 x B2
	D3DXVec3Cross(&L, &a.u[0], &b.u[2]);
	if (D3DXVec3LengthSq(&L) > FLT_EPSILON)
	{
		D3DXVec3Normalize(&L, &L);
		ra = sum_of_projected_radii(a, L);
		rb = sum_of_projected_radii(b, L);
		penetration = ra + rb - fabsf(L.x * T.x + L.y * T.y + L.z * T.z);
		if (penetration < 0) return 0;
		if (smallest_penetration > penetration)
		{
			smallest_penetration = penetration;
			smallest_axis[0] = 0;
			smallest_axis[1] = 2;
			smallest_case = EDGE_EDGE;
		}
	}
	// Test axis L = A1 x B0
	D3DXVec3Cross(&L, &a.u[1], &b.u[0]);
	if (D3DXVec3LengthSq(&L) > FLT_EPSILON)
	{
		D3DXVec3Normalize(&L, &L);
		ra = sum_of_projected_radii(a, L);
		rb = sum_of_projected_radii(b, L);
		penetration = ra + rb - fabsf(L.x * T.x + L.y * T.y + L.z * T.z);
		if (penetration < 0) return 0;
		if (smallest_penetration > penetration)
		{
			smallest_penetration = penetration;
			smallest_axis[0] = 1;
			smallest_axis[1] = 0;
			smallest_case = EDGE_EDGE;
		}
	}

	// Test axis L = A1 x B1
	D3DXVec3Cross(&L, &a.u[1], &b.u[1]);
	if (D3DXVec3LengthSq(&L) > FLT_EPSILON)
	{
		D3DXVec3Normalize(&L, &L);
		ra = sum_of_projected_radii(a, L);
		rb = sum_of_projected_radii(b, L);
		penetration = ra + rb - fabsf(L.x * T.x + L.y * T.y + L.z * T.z);
		if (penetration < 0) return 0;
		if (smallest_penetration > penetration)
		{
			smallest_penetration = penetration;
			smallest_axis[0] = 1;
			smallest_axis[1] = 1;
			smallest_case = EDGE_EDGE;
		}
	}

	// Test axis L = A1 x B2
	D3DXVec3Cross(&L, &a.u[1], &b.u[2]);
	if (D3DXVec3LengthSq(&L) > FLT_EPSILON)
	{
		D3DXVec3Normalize(&L, &L);
		ra = sum_of_projected_radii(a, L);
		rb = sum_of_projected_radii(b, L);
		penetration = ra + rb - fabsf(L.x * T.x + L.y * T.y + L.z * T.z);
		if (penetration < 0) return 0;
		if (smallest_penetration > penetration)
		{
			smallest_penetration = penetration;
			smallest_axis[0] = 1;
			smallest_axis[1] = 2;
			smallest_case = EDGE_EDGE;
		}
	}

	// Test axis L = A2 x B0
	D3DXVec3Cross(&L, &a.u[2], &b.u[0]);
	if (D3DXVec3LengthSq(&L) > FLT_EPSILON)
	{
		D3DXVec3Normalize(&L, &L);
		ra = sum_of_projected_radii(a, L);
		rb = sum_of_projected_radii(b, L);
		penetration = ra + rb - fabsf(L.x * T.x + L.y * T.y + L.z * T.z);
		if (penetration < 0) return 0;
		if (smallest_penetration > penetration)
		{
			smallest_penetration = penetration;
			smallest_axis[0] = 2;
			smallest_axis[1] = 0;
			smallest_case = EDGE_EDGE;
		}
	}

	// Test axis L = A2 x B1
	D3DXVec3Cross(&L, &a.u[2], &b.u[1]);
	if (D3DXVec3LengthSq(&L) > FLT_EPSILON)
	{
		D3DXVec3Normalize(&L, &L);
		ra = sum_of_projected_radii(a, L);
		rb = sum_of_projected_radii(b, L);
		penetration = ra + rb - fabsf(L.x * T.x + L.y * T.y + L.z * T.z);
		if (penetration < 0) return 0;
		if (smallest_penetration > penetration)
		{
			smallest_penetration = penetration;
			smallest_axis[0] = 2;
			smallest_axis[1] = 1;
			smallest_case = EDGE_EDGE;
		}
	}

	// Test axis L = A2 x B2
	D3DXVec3Cross(&L, &a.u[2], &b.u[2]);
	if (D3DXVec3LengthSq(&L) > FLT_EPSILON)
	{
		D3DXVec3Normalize(&L, &L);
		ra = sum_of_projected_radii(a, L);
		rb = sum_of_projected_radii(b, L);
		penetration = ra + rb - fabsf(L.x * T.x + L.y * T.y + L.z * T.z);
		if (penetration < 0) return 0;
		if (smallest_penetration > penetration)
		{
			smallest_penetration = penetration;
			smallest_axis[0] = 2;
			smallest_axis[1] = 2;
			smallest_case = EDGE_EDGE;
		}
	}

	//assert(smallest_penetration < FLT_MAX);
	//assert(smallest_penetration > FLT_EPSILON);

	// Since no separating axis is found, the OBBs must be intersecting
	return (smallest_penetration < FLT_MAX && smallest_penetration > FLT_EPSILON) ? 1 : 0;
}
INT generate_contact_box_box(Box *b0, Box *b1, std::vector<Contact> *contacts, FLOAT restitution)
{
	D3DXMATRIX m;
	D3DXMatrixRotationQuaternion(&m, &b0->orientation);
	OBB obb0;
	obb0.c = b0->position;
	obb0.u[0].x = m._11; obb0.u[0].y = m._12; obb0.u[0].z = m._13;
	obb0.u[1].x = m._21; obb0.u[1].y = m._22; obb0.u[1].z = m._23;
	obb0.u[2].x = m._31; obb0.u[2].y = m._32; obb0.u[2].z = m._33;
	obb0.e = b0->half_size;

	D3DXMatrixRotationQuaternion(&m, &b1->orientation);
	OBB obb1;
	obb1.c = b1->position;
	obb1.u[0].x = m._11; obb1.u[0].y = m._12; obb1.u[0].z = m._13;
	obb1.u[1].x = m._21; obb1.u[1].y = m._22; obb1.u[1].z = m._23;
	obb1.u[2].x = m._31; obb1.u[2].y = m._32; obb1.u[2].z = m._33;
	obb1.e = b1->half_size;

	//��sat_obb_obb�֐������L�����擾�ł���悤�ɉ�������
	FLOAT smallest_penetration = FLT_MAX;	//�ŏ��߂荞�ݗ�
	INT smallest_axis[2];	//�ŏ��߂荞�ݗʂ𓾂��������̍쐬�Ɏg�p�����eOBB�̃��[�J�����ԍ�(0-2)
	SAT_TYPE smallest_case;	//�Փ˂̎�� enum SAT_TYPE { POINTA_FACETB, POINTB_FACETA, EDGE_EDGE };
	if (!sat_obb_obb(obb0, obb1, smallest_penetration, smallest_axis, smallest_case)) return 0;

	//�@���L�R�[�h�𗝉�����
	//obb1�̒��_��obb0�̖ʂƏՓ˂����ꍇ
	if (smallest_case == POINTB_FACETA)
	{
		D3DXVECTOR3 d = obb1.c - obb0.c;	//obb0����obb1�ւ̑��Έʒu
		D3DXVECTOR3 n = obb0.u[smallest_axis[0]];	//obb0�̏Փ˖ʂ̖@���ƕ��s��obb0�̃��[�J�����x�N�g��
		if (D3DXVec3Dot(&n, &d) > 0)	//obb0��obb1�̈ʒu�֌W���Փ˖ʂ̖@���x�N�g�������肷��
		{
			n = n * -1.0f;
		}
		D3DXVec3Normalize(&n, &n);

		//�ڐG�_(p)��obb1��8���_�̂����̂ǂꂩ
		D3DXVECTOR3 p = obb1.e;	//obb1�̊e�ӂ̒����́Aobb1�̏d�S����ڐG�_(p)�ւ̑��Έʒu�̎肪����ɂȂ�
		//obb0��obb1�̈ʒu�֌W(d)���ڐG�_(p)�����߂�
		if (D3DXVec3Dot(&obb1.u[0], &d) > 0) p.x = -p.x;
		if (D3DXVec3Dot(&obb1.u[1], &d) > 0) p.y = -p.y;
		if (D3DXVec3Dot(&obb1.u[2], &d) > 0) p.z = -p.z;
		//���[���h��Ԃ֍��W�ϊ�
		rotate_vector_by_quaternion(&p, b1->orientation, p);
		p += b1->position;

		//Contact�I�u�W�F�N�g�𐶐����A�S�Ẵ����o�ϐ��ɒl���Z�b�g���A�R���e�i(contacts)�ɒǉ�����
		Contact contact;
		contact.normal = n;
		contact.point = p;
		contact.penetration = smallest_penetration;
		contact.body[0] = b0;
		contact.body[1] = b1;
		contact.restitution = restitution;
		contacts->push_back(contact);
	}
	//�Aobb0�̒��_��obb1�̖ʂƏՓ˂����ꍇ�i�@���Q�l�Ɏ�������j
	//Contact�I�u�W�F�N�g�𐶐����A�S�Ẵ����o�ϐ��ɒl���Z�b�g���A�R���e�i(contacts)�ɒǉ�����
	else if (smallest_case == POINTA_FACETB)
	{
		D3DXVECTOR3 d = obb0.c - obb1.c;
		D3DXVECTOR3 n = obb1.u[smallest_axis[1]];
		if (D3DXVec3Dot(&n, &d) > 0)
		{
			n = n * -1.0f;
		}
		D3DXVec3Normalize(&n, &n);

		D3DXVECTOR3 p = obb0.e;
		if (D3DXVec3Dot(&obb0.u[0], &d) > 0) p.x = -p.x;
		if (D3DXVec3Dot(&obb0.u[1], &d) > 0) p.y = -p.y;
		if (D3DXVec3Dot(&obb0.u[2], &d) > 0) p.z = -p.z;

		rotate_vector_by_quaternion(&p, b0->orientation, p);
		p += b0->position;

		Contact contact;
		contact.normal = n;
		contact.point = p;
		contact.penetration = smallest_penetration;
		contact.body[0] = b1;
		contact.body[1] = b0;
		contact.restitution = restitution;
		contacts->push_back(contact);
	}
	//�Bobb0�̕ӂ�obb1�̕ӂƏՓ˂����ꍇ
	//Contact�I�u�W�F�N�g�𐶐����A�S�Ẵ����o�ϐ��ɒl���Z�b�g���A�R���e�i(contacts)�ɒǉ�����
	else if (smallest_case == EDGE_EDGE)
	{
		D3DXVECTOR3 d = obb1.c - obb0.c;
		D3DXVECTOR3 n;
		D3DXVec3Cross(&n, &obb0.u[smallest_axis[0]], &obb1.u[smallest_axis[1]]);
		D3DXVec3Normalize(&n, &n);
		if (D3DXVec3Dot(&n, &d) > 0)
		{
			n = n * -1.0f;
		}

		D3DXVECTOR3 p[2] = { obb0.e, obb1.e };
		{
			if (D3DXVec3Dot(&obb0.u[0], &n) > 0) p[0].x = -p[0].x;
			if (D3DXVec3Dot(&obb0.u[1], &n) > 0) p[0].y = -p[0].y;
			if (D3DXVec3Dot(&obb0.u[2], &n) > 0) p[0].z = -p[0].z;
			p[0][smallest_axis[0]] = 0;
			rotate_vector_by_quaternion(&p[0], b0->orientation, p[0]);
			p[0] += b0->position;

			//_DDM::I().AddCross(p[0], 1);
			//_DDM::I().AddLine(b0->position, b0->position + smallest_penetration * 100 * obb0.u[smallest_axis[0]]);

			if (D3DXVec3Dot(&obb1.u[0], &n) < 0) p[1].x = -p[1].x;
			if (D3DXVec3Dot(&obb1.u[1], &n) < 0) p[1].y = -p[1].y;
			if (D3DXVec3Dot(&obb1.u[2], &n) < 0) p[1].z = -p[1].z;
			p[1][smallest_axis[1]] = 0;
			rotate_vector_by_quaternion(&p[1], b1->orientation, p[1]);
			p[1] += b1->position;

			//_DDM::I().AddCross(p[1], 1);
			//_DDM::I().AddLine(b1->position, b1->position + smallest_penetration * 100 * obb1.u[smallest_axis[1]]);
		}

		Contact contact;
		contact.normal = n;
		contact.point = (p[0] + p[1]) * 0.5f;
		contact.penetration = smallest_penetration;
		contact.body[0] = b0;
		contact.body[1] = b1;
		contact.restitution = restitution;
		contacts->push_back(contact);
	}
	else assert(0);

	return 1;
}

void Contact::resolve()
{
	assert(penetration > 0);

	//David Baraff[1997] An Introduction to Physically Based Modeling:Rigid Body Simulation II - Nonpenetration Constraints pp.40-47
	//Baraff[1997]�̎�(8-1)(8-2)(8-3)����ՓˑO�̑��Α��x(vrel)�����߂�
	FLOAT vrel = 0;

	//(8-1)
	D3DXVECTOR3 pdota;
	D3DXVec3Cross(&pdota, &body[0]->angular_velocity, &(point - body[0]->position));
	pdota += body[0]->linear_velocity;

	//(8-2)
	D3DXVECTOR3 pdotb;
	D3DXVec3Cross(&pdotb, &body[1]->angular_velocity, &(point - body[1]->position));
	pdotb += body[1]->linear_velocity;

	//(8-3)
	vrel = D3DXVec3Dot(&normal, &(pdota - pdotb));

	//Baraff[1997]�̎�(8-18)�̕��q(numerator)�����߂�
	FLOAT numerator = 0;
	numerator = -(1 + restitution) * vrel;

	//Baraff[1997]�̎�(8-18)�̕���(denominator)�����߂�
	FLOAT denominator = 0;
	FLOAT term1 = body[0]->inverse_mass();
	FLOAT term2 = body[1]->inverse_mass();
	D3DXVECTOR3 ra = point - body[0]->position;
	D3DXVECTOR3 rb = point - body[1]->position;
	D3DXVECTOR3 ta, tb;
	D3DXVec3Cross(&ta, &ra, &normal);
	D3DXVec3Cross(&tb, &rb, &normal);
	D3DXVec3TransformCoord(&ta, &ta, &body[0]->inverse_inertia_tensor());
	D3DXVec3TransformCoord(&tb, &tb, &body[1]->inverse_inertia_tensor());
	D3DXVec3Cross(&ta, &ta, &ra);
	D3DXVec3Cross(&tb, &tb, &rb);
	FLOAT term3 = D3DXVec3Dot(&normal, &ta);
	FLOAT term4 = D3DXVec3Dot(&normal, &tb);
	denominator = term1 + term2 + term3 + term4;

	//Baraff[1997]�̎�(8-18)�̌���(j)�����߂�
	FLOAT j = 0;
	j = numerator / denominator;

	//Baraff[1997]�̎�(8-12)���e���̂̕��i���x(linear_velocity)�Ɗp���x(angular_velocity)���X�V����
	D3DXVECTOR3 impulse = j * normal;

	D3DXVECTOR3 friction(0, 0, 0);	//���C��
	FLOAT cof = 0.6f;		//���C�W���iCoefficient of friction�j
	//�@�����C�́iDynamic Friction�j��\������
	D3DXVECTOR3 vta = pdota - D3DXVec3Dot(&normal, &pdota) * normal;	//�Փ˓_A�̑��x�ipdota�j�̏Փ˖ʐڐ�����
	D3DXVECTOR3 vtb = pdotb - D3DXVec3Dot(&normal, &pdotb) * normal;	//�Փ˓_B�̑��x�ipdotb�j�̏Փ˖ʐڐ�����
	D3DXVECTOR3 vt = vta - vtb;	//�Փ˓_AB�̏Փ˖ʐڐ������̑��Α��x�i���ׂ�����j
	FLOAT vrel_t = D3DXVec3Length(&vt);
	if (vrel_t > fabsf(vrel) * cof)	//�Փ˓_AB�̏Փ˖ʖ@�������̑��Α��x�̑傫���ivrel�j�ƏՓ˓_AB�̏Փ˖ʐڐ������̑��Α��x�̑傫���ivrel_t�j�̔�r
	{
		D3DXVec3Normalize(&friction, &-vt); //���C�͂͂��ׂ�����̋t�����ɍ�p����
		friction *= j * cof;	//���C�͂͏Փ˂̑傫���Ɩ��C�W���ɔ�Ⴗ��
	}
	impulse += friction;	//���͂ɕ␳��^����

	body[0]->linear_velocity += impulse * body[0]->inverse_mass();
	D3DXVec3Cross(&ta, &ra, &impulse);
	D3DXVec3TransformCoord(&ta, &ta, &body[0]->inverse_inertia_tensor());
	body[0]->angular_velocity += ta;

	body[1]->linear_velocity -= impulse * body[1]->inverse_mass();
	D3DXVec3Cross(&tb, &rb, &impulse);
	D3DXVec3TransformCoord(&tb, &tb, &body[1]->inverse_inertia_tensor());
	body[1]->angular_velocity -= tb;

	//�߂荞�ݗʂ̉���
	body[0]->position += penetration * body[1]->inertial_mass / (body[0]->inertial_mass + body[1]->inertial_mass) * normal;
	body[1]->position -= penetration * body[0]->inertial_mass / (body[0]->inertial_mass + body[1]->inertial_mass) * normal;
}

