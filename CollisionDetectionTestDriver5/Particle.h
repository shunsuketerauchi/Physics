#pragma once

#include <d3dx9.h>
#include <assert.h>

class Particle
{
public:
	FLOAT mass;
	D3DXVECTOR3 position;
	D3DXVECTOR3 velocity;

private:
	D3DXVECTOR3 acceleration;
	D3DXVECTOR3 resultant;

public:
	Particle() : mass(FLT_MAX), position(0, 0, 0), velocity(0, 0, 0),
		acceleration(0, 0, 0), resultant(0, 0, 0) {}

	virtual ~Particle() {}

	void integrate(FLOAT duration)
	{
		assert(mass > 0);

		acceleration = (resultant / mass);
		velocity += acceleration * duration;
		position += velocity * duration;

		resultant = D3DXVECTOR3(0, 0, 0);
	}
	void add_force(CONST D3DXVECTOR3 &force)
	{
		resultant += force;
	}
	const D3DXVECTOR3 &get_acceleration() const
	{
		return acceleration;
	}
	void collide(D3DXVECTOR3 normal, FLOAT restitution, FLOAT penetration)
	{
		assert(penetration > 0);

		D3DXVec3Normalize(&normal, &normal);
		FLOAT v = D3DXVec3Dot(&velocity, &normal);
		if (v < 0)
		{
			velocity += -(restitution + 1) * v * normal;
		}
		position += penetration * normal;
	}
#if 1
	static void collide(Particle *p, Particle *q, FLOAT restitution, FLOAT penetration)
	{
		assert(penetration > 0);

		FLOAT m1 = p->mass;
		FLOAT m2 = q->mass;
		FLOAT e = restitution;
		FLOAT d = penetration;

		D3DXVECTOR3 n = q->position - p->position;
		D3DXVec3Normalize(&n, &n);

		FLOAT v1 = D3DXVec3Dot(&p->velocity, &n);
		FLOAT v2 = D3DXVec3Dot(&q->velocity, &n);
		if (v1 - v2 > 0)
		{
			p->velocity += ((m1 * v1 + m2 * v2 + e * m2 * (v2 - v1)) / (m1 + m2) - v1) * n;
			q->velocity += ((m1 * v1 + m2 * v2 + e * m1 * (v1 - v2)) / (m1 + m2) - v2) * n;
		}
		p->position -= m2 / (m1 + m2) * d * n;
		q->position += m1 / (m1 + m2) * d * n;
	}
#else
	static void collide( Particle *p, Particle *q, FLOAT restitution, FLOAT penetration )
	{
		//ゲーム制作者のための物理シミュレーション 剛体編 p.78-81
		D3DXVECTOR3 Va = p->velocity;
		D3DXVECTOR3 Vb = q->velocity;
		FLOAT ma = p->mass;
		FLOAT mb = q->mass;
		FLOAT e = restitution;
		FLOAT d = penetration;

		D3DXVECTOR3 n = q->position - p->position;
		D3DXVec3Normalize( &n, &n );

		D3DXVECTOR3 Vab = Vb - Va;
		//D3DXVECTOR3 Vn = D3DXVec3Dot( &Vab, &n ) * n;
		//D3DXVECTOR3 Vt = Vab - Vn;

		//if( D3DXVec3Dot( &Vab, &n ) < 0 )
		{
			FLOAT cd = 30.0f;
			FLOAT c = ma * mb / ( ma + mb ) * ( ( 1 + e ) * D3DXVec3Dot( &Vab, &n ) - cd * d );
			//FLOAT c = ma * mb / ( ma + mb ) * ( ( 1 + e ) * D3DXVec3Dot( &Vab, &n ) );

			p->velocity += c * n / ma;
			q->velocity -= c * n / mb;
		}
		//p->position -= mb / (ma + mb) * d * n;
		//q->position += ma / (ma + mb) * d * n;
	}
#endif

	struct Constraint
	{
		Particle *p;
		Particle *q;
		FLOAT length;

		Constraint(Particle *p, Particle *q) : p(p), q(q), length(D3DXVec3Length(&(q->position - p->position))) {}

		virtual void resolve(FLOAT duration) = 0;
	};
	struct Rod : public Constraint
	{
		Rod(Particle *p, Particle *q) : Constraint(p, q) {}
		void resolve(FLOAT duration)
		{
			assert(length > 0);
			D3DXVECTOR3 n = q->position - p->position;
			FLOAT l = D3DXVec3Length(&n);
			FLOAT penetration = length - l;
			if (fabsf(penetration) > FLT_EPSILON) 
			{
				D3DXVec3Normalize(&n, &n);
				p->position -= q->mass / (p->mass + q->mass) * penetration * n;
				q->position += p->mass / (p->mass + q->mass) * penetration * n;
			}
		}
	};
	struct Cable : public Constraint
	{
		Cable(Particle *p, Particle *q) : Constraint(p, q) {}
		void resolve(FLOAT duration)
		{
			assert(length > 0);
			D3DXVECTOR3 n = q->position - p->position;
			FLOAT l = D3DXVec3Length(&n);
			if (l - length > FLT_EPSILON)
			{
				D3DXVec3Normalize(&n, &n);
				p->position -= q->mass / (p->mass + q->mass) * (length - l) * n;
				q->position += p->mass / (p->mass + q->mass) * (length - l) * n;
			}
		}
	};


};





