# Modbus DLL

## c++ dll
- 목적: C++에서 만든 DLL을 C 방식으로 export 해서 C# WPF에서 import 해서 사용
- 구성:
	- ModbusClient.dll: Modbus TCP 클라이언트 기능
	- ModbusServer.dll: Modbus TCP 서버 기능
- 빌드 환경:
	- CMake
	- libmodbus
        - vcpkg로 설치시 x64-windows.cmake 수정:   
            - set(VCPKG_TARGET_ARCHITECTURE x64)  
            set(VCPKG_CRT_LINKAGE dynamic)  
            set(VCPKG_LIBRARY_LINKAGE dynamic)  
            **set(VCPKG_C_FLAGS "/DFD_SETSIZE=2048")**  
            **set(VCPKG_CXX_FLAGS "/DFD_SETSIZE=2048")**  
        - dll 위치 C:/vcpkg/installed/x64-windows/bin/modbus-5.dll

	- Windows

- 클라이언트 DLL: 다른 Modbus 장비에 접속해서 읽기/쓰기
- 서버 DLL: 내 프로그램이 Modbus 장비처럼 동작

기본 반환값 규칙

- 성공: 1
- 실패: 0
- 예외: ModbusClient_GetLastErrno는 에러 코드를 반환하고, handle이 잘못되면 -1 반환
- 문자열을 받는 함수는 호출하는 쪽에서 buffer를 미리 만들어서 넘겨야 함

기본 사용 순서

1. Create로 객체 생성
2. IP, Port 설정
3. 필요한 작업 수행
4. 종료 시 Destroy 호출

간단한 흐름 예시

### Client
1. ModbusClient_Create
2. ModbusClient_SetIp
3. ModbusClient_SetPort
4. ModbusClient_Connect
5. Read/Write 함수 호출
6. ModbusClient_Disconnect
7. ModbusClient_Destroy

### Server
1. ModbusServer_Create
2. ModbusServer_SetIp
3. ModbusServer_SetPort
4. ModbusServer_AddAddress
5. ModbusServer_SetValueAt
6. ModbusServer_Start
7. 필요 시 값 조회/변경
8. ModbusServer_Stop
9. ModbusServer_Destroy

# api 문서 - ModbusClient.dll

클라이언트는 Modbus TCP 서버에 접속해서 coil, input, register를 읽거나 씁니다.

## 생성 / 종료

##### void* ModbusClient_Create()
- 설명: 클라이언트 객체 생성
- 사용 시점: 제일 먼저 호출
- 반환값: 생성된 핸들, 실패 시 nullptr

##### void ModbusClient_Destroy(void* handle)
- 설명: 클라이언트 객체 메모리 해제
- 사용 시점: 마지막에 반드시 호출

## 연결 설정

##### int ModbusClient_SetIp(void* handle, const char* ip)
- 설명: 접속할 서버 IP 설정
- 예: 127.0.0.1
- 주의: 이미 연결 중이면 실패할 수 있음

##### int ModbusClient_GetIp(void* handle, char* buffer, int bufferSize)
- 설명: 현재 설정된 IP 문자열 가져오기
- 사용법: 충분한 크기의 buffer 전달
- 예: char buffer[64]

##### int ModbusClient_SetPort(void* handle, uint16_t port)
- 설명: 접속할 포트 설정
- 예: 502

##### uint16_t ModbusClient_GetPort(void* handle)
- 설명: 현재 설정된 포트 조회

## 연결 상태

##### int ModbusClient_Connect(void* handle)
- 설명: 서버 연결 시도
- 성공 시: 1
- 실패 시: 0

##### int ModbusClient_Disconnect(void* handle)
- 설명: 서버 연결 해제

##### int ModbusClient_IsConnected(void* handle)
- 설명: 현재 연결 여부 확인
- 반환값: 연결됨 1, 아니면 0

##### int ModbusClient_GetLastErrno(void* handle)
- 설명: 마지막 에러 코드 조회
- 사용 시점: Connect 또는 Read/Write 실패 직후 확인

##### int ModbusClient_GetLastErrorString(void* handle, char* buffer, int bufferSize)
- 설명: 마지막 에러 메시지 문자열 조회
- 사용법: buffer를 미리 생성해서 전달

## 읽기 함수

##### int ModbusClient_ReadCoils(void* handle, int address, int count, uint8_t* out_bits)
- 설명: Coil 값 여러 개 읽기
- address: 시작 주소
- count: 읽을 개수
- out_bits: 결과를 담을 배열, 크기는 count 이상 필요
- 결과값: 각 원소는 0 또는 1

##### int ModbusClient_ReadDiscreteInputs(void* handle, int address, int count, uint8_t* out_bits)
- 설명: Discrete Input 값 여러 개 읽기
- out_bits: 결과 배열

##### int ModbusClient_ReadInputRegisters(void* handle, int address, int count, uint16_t* out_regs)
- 설명: Input Register 값 여러 개 읽기
- out_regs: 결과 배열, 크기는 count 이상 필요

##### int ModbusClient_ReadHoldingRegisters(void* handle, int address, int count, uint16_t* out_regs)
- 설명: Holding Register 값 여러 개 읽기
- out_regs: 결과 배열, 크기는 count 이상 필요

## 쓰기 함수

##### int ModbusClient_WriteCoil(void* handle, int address, int value)
- 설명: Coil 1개 쓰기
- value: 0 또는 1

##### int ModbusClient_WriteCoils(void* handle, int address, int count, const uint8_t* bits)
- 설명: Coil 여러 개 쓰기
- bits: 쓸 값 배열

##### int ModbusClient_WriteHoldingRegister(void* handle, int address, int value)
- 설명: Holding Register 1개 쓰기
- value: 일반적으로 0~65535 사용

##### int ModbusClient_WriteHoldingRegisters(void* handle, int address, int count, const uint16_t* regs)
- 설명: Holding Register 여러 개 쓰기
- regs: 쓸 값 배열

## Client 간단 예시

```cpp
void* client = ModbusClient_Create();

ModbusClient_SetIp(client, "127.0.0.1");
ModbusClient_SetPort(client, 502);

if (ModbusClient_Connect(client) == 1)
{
		uint16_t values[2] = {0, 0};
		if (ModbusClient_ReadHoldingRegisters(client, 40001, 2, values) == 1)
		{
				// values 사용
		}

		ModbusClient_Disconnect(client);
}

ModbusClient_Destroy(client);
```

# api 문서 - ModbusClient.dll

서버는 내 프로그램이 Modbus TCP 서버처럼 동작하도록 만듭니다.

주소 타입(type) 값은 아래와 같습니다.

- 0: Coil
- 1: Discrete Input
- 2: Input Register
- 3: Holding Register

## 생성 / 종료 / 실행

##### void* ModbusServer_Create()
- 설명: 서버 객체 생성

##### void ModbusServer_Destroy(void* handle)
- 설명: 서버 객체 해제

##### int ModbusServer_Start(void* handle)
- 설명: 서버 시작
- 사용 전 권장 순서: IP, Port, Address 설정 먼저 진행

##### int ModbusServer_Stop(void* handle)
- 설명: 서버 중지

##### int ModbusServer_IsRunning(void* handle)
- 설명: 서버 실행 여부 확인

## 서버 설정

##### int ModbusServer_SetIp(void* handle, const char* ip)
- 설명: 서버 바인딩 IP 설정
- 예: 0.0.0.0 또는 127.0.0.1

##### int ModbusServer_GetIp(void* handle, char* buffer, int bufferSize)
- 설명: 현재 설정된 IP 조회

##### int ModbusServer_SetPort(void* handle, uint16_t port)
- 설명: 서버 포트 설정
- 예: 502

##### uint16_t ModbusServer_GetPort(void* handle)
- 설명: 현재 포트 조회

## 주소 관리

##### int ModbusServer_AddAddress(void* handle, int type, int address)
- 설명: 서버가 관리할 주소 추가
- 예:
	- type 0, address 1
	- type 3, address 40001

##### int ModbusServer_RemoveAddress(void* handle, int type, int index)
- 설명: 등록한 주소를 index 기준으로 삭제

##### int ModbusServer_GetAddressCount(void* handle, int type)
- 설명: 특정 type에 등록된 주소 개수 조회

##### int ModbusServer_GetAddressAt(void* handle, int type, int index, int* outAddress)
- 설명: index 위치의 주소값 조회

##### int ModbusServer_SetAddressAt(void* handle, int type, int index, int address)
- 설명: index 위치의 주소값 변경

## 값 관리

##### int ModbusServer_GetValueAt(void* handle, int type, int index, int* outValue)
- 설명: index 위치의 현재 값 조회

##### int ModbusServer_SetValueAt(void* handle, int type, int index, int value)
- 설명: index 위치의 값 변경
- 주의:
	- Coil, Discrete Input은 0 또는 1
	- Input Register, Holding Register는 0~65535 범위 권장

## Server 간단 예시

```cpp
void* server = ModbusServer_Create();

ModbusServer_SetIp(server, "0.0.0.0");
ModbusServer_SetPort(server, 502);

ModbusServer_AddAddress(server, 3, 40001);
ModbusServer_SetValueAt(server, 3, 0, 1234);

ModbusServer_Start(server);

// 실행 중 필요하면 값 변경
ModbusServer_SetValueAt(server, 3, 0, 4321);

ModbusServer_Stop(server);
ModbusServer_Destroy(server);
```

## TODO list

- 함수 선언용 header 파일 정리
- 에러 코드 표 정리
