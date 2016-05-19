#include "Particle.h"
#include "RigidBody.h"

class CollisionDetectionTestDriver : public Scene
{
	LPD3DXMESH box, sphere;

	Sphere *sphere_body[3];
	Box *box_body[3];
	Plane *plane_body;
	std::vector<Contact> contacts;

public:
	CollisionDetectionTestDriver(LPDIRECT3DDEVICE9 d3dd) : sphere(0), box(0)
	{
		D3DXCreateBox(d3dd, 1, 1, 1, &box, 0);
		D3DXCreateSphere(d3dd, 1.0f, 12, 12, &sphere, 0);

		sphere_body[0] = new Sphere(2, 0.1f);
		sphere_body[0]->position = D3DXVECTOR3(1, 10, 0);
		sphere_body[1] = new Sphere(2, 0.1f);
		sphere_body[1]->position = D3DXVECTOR3(1, 20, 0);
		sphere_body[2] = new Sphere(2, 0.1f);
		sphere_body[2]->position = D3DXVECTOR3(1, 30, 0);

		box_body[0] = new Box(D3DXVECTOR3(2.0f, 2.0f, 2.0f), 0.1f);
		box_body[0]->position = D3DXVECTOR3(5, 5, 10);
		box_body[1] = new Box(D3DXVECTOR3(2.0f, 2.0f, 2.0f), 0.1f);
		box_body[1]->position = D3DXVECTOR3(5, 15, 10);
		box_body[2] = new Box(D3DXVECTOR3(2.0f, 2.0f, 2.0f), 0.1f);
		box_body[2]->position = D3DXVECTOR3(5, 25, 10);

		plane_body = new Plane(D3DXVECTOR3(0, 1, 0), -2);
	}
	~CollisionDetectionTestDriver()
	{
		if (box) box->Release();
		if (sphere) sphere->Release();

		for (int i = 0; i < 3; i++)
		{
			if (sphere_body[i]) delete sphere_body[i];
		}
		for (int i = 0; i < 3; i++)
		{
			if (box_body[i]) delete box_body[i];
		}
		if (plane_body) delete plane_body;
	}
	void Update(FLOAT duration)
	{
		Scene::Update(duration);
		static FLOAT pitch = 0, roll = 0;
		if (GetKeyState('W') < 0) pitch += duration * 0.5f;
		if (GetKeyState('S') < 0) pitch -= duration * 0.5f;
		if (GetKeyState('A') < 0) roll += duration * 0.5f;
		if (GetKeyState('D') < 0) roll -= duration * 0.5f;
		D3DXQuaternionRotationYawPitchRoll(&plane_body->orientation, 0, pitch, roll);

		if (GetKeyState(VK_LEFT) < 0) box_body[0]->position.x -= 0.1f;
		if (GetKeyState(VK_RIGHT) < 0) box_body[0]->position.x += 0.1f;
		if (GetKeyState(VK_UP) < 0) box_body[0]->position.z += 0.1f;
		if (GetKeyState(VK_DOWN) < 0) box_body[0]->position.z -= 0.1f;

		D3DXVECTOR3 g(0, -9.8f, 0);
		for (int i = 0; i < 3; i++)
		{
			sphere_body[i]->add_force(sphere_body[i]->inertial_mass * g);
			box_body[i]->add_force(box_body[i]->inertial_mass * g);
		}

		for (int i = 0; i < 3; i++)
		{
			sphere_body[i]->integrate(duration);
			box_body[i]->integrate(duration);
		}
		plane_body->integrate(duration);

		for (int i = 0; i < 3; i++)
		{
			generate_contact_sphere_plane(sphere_body[i], plane_body, &contacts, 0.4f);
			for (int j = i + 1; j < 3; j++)
			{
				generate_contact_sphere_sphere(sphere_body[i], sphere_body[j], &contacts, 0.4f);
			}
			for (int j = 0; j < 3; j++)
			{
				generate_contact_sphere_box(sphere_body[i], box_body[j], &contacts, 0.4f);
			}
		}
		for (int i = 0; i < 3; i++)
		{
			generate_contact_box_plane(box_body[i], plane_body, &contacts, 0.4f);
			for (int j = i + 1; j < 3; j++)
			{
				generate_contact_box_box(box_body[i], box_body[j], &contacts, 0.4f);
			}
		}

		for (unsigned i = 0; i < contacts.size(); i++)
		{
			contacts[i].resolve();
			_DDM::I().AddCross(contacts[i].point, 1, _DDM::WHITE, 0);
			_DDM::I().AddLine(contacts[i].point, contacts[i].point + contacts[i].penetration * 100 * contacts[i].normal, _DDM::RED, 0);
		}
		contacts.clear();
	}

	void Render(LPDIRECT3DDEVICE9 d3dd)
	{
		{
			D3DXMATRIX V;
			D3DXMatrixLookAtLH(&V,
				//&D3DXVECTOR3( 10.0f, 20.0f, -15.0f ),
				&(comera_position * comera_distance),
				&D3DXVECTOR3(0.0f, 0.0f, 0.0f),
				&D3DXVECTOR3(0.0f, 1.0f, 0.0f)
				);
			d3dd->SetTransform(D3DTS_VIEW, &V);

			D3DXMATRIX P;
			RECT rect;
			if (GetClientRect(GetActiveWindow(), &rect))
			{
				D3DXMatrixPerspectiveFovLH(&P, 60 * 0.01745f, (FLOAT)rect.right / rect.bottom, 1.0f, 1000.0f);
				d3dd->SetTransform(D3DTS_PROJECTION, &P);
			}

			D3DLIGHT9 l = { D3DLIGHT_DIRECTIONAL, 0 };
			l.Ambient = l.Diffuse = D3DXCOLOR(0.8f, 0.8f, 0.8f, 1.0f);
			l.Direction = D3DXVECTOR3(0, 0, 1);
			d3dd->SetLight(0, &l);
			d3dd->LightEnable(0, TRUE);
		}

		D3DMATERIAL9 m = { 0 };

		D3DXMATRIX M, R, S, T;

		m.Ambient = m.Diffuse = D3DXCOLOR(0.6f, 0.6f, 0.6f, 0.0f);
		d3dd->SetMaterial(&m);

		for (int i = 0; i < 3; i++)
		{
			m.Ambient = m.Diffuse = D3DXCOLOR(0.6f, 0.1f, 0.1f, 0.0f);
			d3dd->SetMaterial(&m);
			//d3dd->SetRenderState( D3DRS_FILLMODE, D3DFILL_SOLID ); 
			d3dd->SetRenderState(D3DRS_FILLMODE, D3DFILL_WIREFRAME);
			RenderRigidBody(d3dd, sphere_body[i]);
		}

		for (int i = 0; i < 3; i++)
		{
			m.Ambient = m.Diffuse = D3DXCOLOR(0.1f, 0.1f, 0.6f, 0.0f);
			d3dd->SetMaterial(&m);
			//d3dd->SetRenderState( D3DRS_FILLMODE, D3DFILL_SOLID ); 
			d3dd->SetRenderState(D3DRS_FILLMODE, D3DFILL_WIREFRAME);
			RenderRigidBody(d3dd, box_body[i]);
		}

		m.Ambient = m.Diffuse = D3DXCOLOR(0.6f, 0.6f, 0.0f, 0.0f);
		d3dd->SetMaterial(&m);
		d3dd->SetRenderState(D3DRS_FILLMODE, D3DFILL_SOLID);
		//d3dd->SetRenderState(D3DRS_FILLMODE, D3DFILL_WIREFRAME);
		RenderRigidBody(d3dd, plane_body);

		_DDM::I().DrawQ(d3dd);
	}
	void RenderRigidBody(LPDIRECT3DDEVICE9 d3dd, RigidBody *body)
	{
		D3DXMATRIX M, R, S, T;

		Box *box_shape = dynamic_cast<Box *>(body);
		if (box_shape)
		{
			D3DXMatrixScaling(&S, box_shape->half_size.x * 2, box_shape->half_size.y * 2, box_shape->half_size.z * 2);
			D3DXMatrixRotationQuaternion(&R, &box_shape->orientation);
			M = S * R;
			M._41 = box_shape->position.x;
			M._42 = box_shape->position.y;
			M._43 = box_shape->position.z;
			d3dd->SetTransform(D3DTS_WORLD, &M);
			box->DrawSubset(0);
		}
		Sphere *sphere_shape = dynamic_cast<Sphere *>(body);
		if (sphere_shape)
		{
			FLOAT r = sphere_shape->r;
			D3DXMatrixScaling(&S, r, r, r);
			D3DXMatrixRotationQuaternion(&R, &body->orientation);
			M = S * R;
			M._41 = body->position.x;
			M._42 = body->position.y;
			M._43 = body->position.z;
			d3dd->SetTransform(D3DTS_WORLD, &M);
			sphere->DrawSubset(0);
		}
		Plane *plane_shape = dynamic_cast<Plane *>(body);
		if (plane_shape)
		{
			D3DXMatrixScaling(&S, 50, 0, 50);
			D3DXMatrixRotationQuaternion(&R, &body->orientation);
			M = S * R;
			M._41 = body->position.x;
			M._42 = body->position.y;
			M._43 = body->position.z;
			d3dd->SetTransform(D3DTS_WORLD, &M);
			box->DrawSubset(0);
		}
	}
};

