#pragma comment( lib, "d3d9.lib" )
#if defined( DEBUG ) || defined( _DEBUG )
#pragma comment( lib, "d3dx9d.lib" )
#else
#pragma comment( lib, "d3dx9.lib" )
#endif
#pragma comment( lib, "winmm.lib" )

#define NOMINMAX
#include <windows.h>
#include <assert.h>

#include <d3dx9.h>

#include <iostream>
#include <io.h>
#include <fcntl.h>

#include <time.h>

#include "DebugDrawManager.h"

POINTS mouse_dragged;
BOOL mouse_captured = FALSE;
LONG mouse_wheel = 0;
class Scene
{
public:
	D3DXVECTOR3 comera_position;
	FLOAT comera_distance;

	Scene() : comera_position(10, 20, -15), comera_distance(2.5f) {}
	virtual ~Scene() {};
	virtual void Update(FLOAT duration)
	{
		if (mouse_captured)
		{
			SHORT backlash = 32;
			FLOAT sensitivity = 0.02f;
			D3DXMATRIX R;
			FLOAT yaw = 0, pitch = 0, roll = 0;
			if (mouse_dragged.x > backlash) yaw += sensitivity;
			else if (mouse_dragged.x < -backlash) yaw -= sensitivity;

			if (mouse_dragged.y > backlash) pitch += sensitivity;
			else if (mouse_dragged.y < -backlash) pitch -= sensitivity;
			D3DXMatrixRotationYawPitchRoll(&R, yaw, pitch, roll);
			D3DXVec3TransformCoord(&comera_position, &comera_position, &R);
		}
		if (mouse_wheel)
		{
			FLOAT sensitivity = 0.1f;
			comera_distance -= (FLOAT)mouse_wheel / 120.0f * sensitivity;
			mouse_wheel = 0;
		}
	}
	virtual void Render(LPDIRECT3DDEVICE9 d3dd) = 0;
};

#include "CollisionDetectionTestDriver.h"

LRESULT WINAPI MsgProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
	static POINTS mouse_origin;
	switch (msg)
	{
	case WM_LBUTTONDOWN:
		mouse_origin = MAKEPOINTS(lParam);
		mouse_captured = true;
		SetCapture(hWnd);
		break;
	case WM_MOUSEMOVE:
		if (mouse_captured)
		{
			SetCursor(LoadCursor(NULL, IDC_CROSS));
			POINTS p = MAKEPOINTS(lParam);
			mouse_dragged.x = p.x - mouse_origin.x;
			mouse_dragged.y = p.y - mouse_origin.y;
		}
		else
		{
			mouse_dragged.x = 0;
			mouse_dragged.y = 0;
			SetCursor(LoadCursor(NULL, IDC_ARROW));
		}
		break;
	case WM_LBUTTONUP:
		SetCursor(LoadCursor(NULL, IDC_ARROW));
		ReleaseCapture();
		mouse_captured = FALSE;
		break;
	case WM_MOUSEWHEEL:
		mouse_wheel = GET_WHEEL_DELTA_WPARAM(wParam);
		break;
	case WM_ACTIVATE:
		mouse_wheel = 0;
		break;
	case WM_KEYDOWN:
		if (wParam == VK_ESCAPE) PostMessage(hWnd, WM_CLOSE, 0, 0);
		break;
	case WM_CREATE:
		//AllocConsole();
		//*stdout = *_fdopen( _open_osfhandle( ( intptr_t )GetStdHandle( STD_OUTPUT_HANDLE ), _O_TEXT ), "w" );
		//setvbuf( stdout, 0, _IONBF, 0 );
		break;
	case WM_DESTROY:
		//FreeConsole();
		PostQuitMessage(0);
		return 0;
	}
	return DefWindowProc(hWnd, msg, wParam, lParam);
}

INT WINAPI wWinMain(HINSTANCE hInst, HINSTANCE, LPWSTR, INT)
{
	_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);

	srand((unsigned int)time((time_t)0));

	WNDCLASSEX wc = { sizeof(WNDCLASSEX), CS_CLASSDC, MsgProc, 0L, 0L, GetModuleHandle(0), 0, 0, 0, 0, L"Physics", 0 };
	RegisterClassEx(&wc);

	HWND hWnd = CreateWindow(L"Physics", L"Physics", WS_OVERLAPPEDWINDOW, 100, 100, 1280, 720, 0, 0, wc.hInstance, 0);

	LPDIRECT3D9 d3d = 0;
	if (!(d3d = Direct3DCreate9(D3D_SDK_VERSION))) return 0;

	LPDIRECT3DDEVICE9 d3dd = 0;
	D3DPRESENT_PARAMETERS d3dpp;
	ZeroMemory(&d3dpp, sizeof(d3dpp));
	d3dpp.Windowed = TRUE;
	d3dpp.SwapEffect = D3DSWAPEFFECT_DISCARD;
	d3dpp.BackBufferFormat = D3DFMT_UNKNOWN;
	d3dpp.EnableAutoDepthStencil = TRUE;
	d3dpp.AutoDepthStencilFormat = D3DFMT_D16;
	if (FAILED(d3d->CreateDevice(D3DADAPTER_DEFAULT, D3DDEVTYPE_HAL, hWnd, D3DCREATE_SOFTWARE_VERTEXPROCESSING, &d3dpp, &d3dd)))
	if (FAILED(d3d->CreateDevice(D3DADAPTER_DEFAULT, D3DDEVTYPE_REF, hWnd, D3DCREATE_SOFTWARE_VERTEXPROCESSING, &d3dpp, &d3dd)))
		assert(0);

	Scene *scene = 0;
	scene = new CollisionDetectionTestDriver(d3dd);

	ShowWindow(hWnd, SW_SHOWDEFAULT);
	UpdateWindow(hWnd);

	MSG msg;
	ZeroMemory(&msg, sizeof(msg));
	while (msg.message != WM_QUIT)
	{
		if (PeekMessage(&msg, 0, 0U, 0U, PM_REMOVE))
		{
			TranslateMessage(&msg);
			DispatchMessage(&msg);
		}
		else
		{
			static DWORD last = timeGetTime();
			DWORD elapse = timeGetTime() - last;
			if (scene && elapse > 0) scene->Update((FLOAT)elapse / 1000.0f);
			last += elapse;

			d3dd->Clear(0, 0, D3DCLEAR_TARGET | D3DCLEAR_ZBUFFER, D3DCOLOR_XRGB(0, 0, 0), 1.0f, 0);
			if (SUCCEEDED(d3dd->BeginScene()))
			{
				if (scene) scene->Render(d3dd);
				d3dd->EndScene();
			}
			d3dd->Present(0, 0, 0, 0);
		}
	}
	if (scene) delete scene;

	if (d3dd) d3dd->Release();
	if (d3d) d3d->Release();

	UnregisterClass(L"Mathematics", wc.hInstance);

	return 0;
}

