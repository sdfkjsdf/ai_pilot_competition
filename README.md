# ai_pilot_competition
2026 AI Pilot Top Gun Challenge에 출판한 대회 코드 

## 저장소 받기

PowerShell에서 다음 명령을 실행합니다.

```powershell
git clone https://github.com/sdfkjsdf/ai_pilot_competition.git
Set-Location .\ai_pilot_competition
```

## 빌드 환경

- Windows x64
- Visual Studio 2022
- Desktop development with C++ 워크로드
- MSVC v143 toolset와 Windows SDK

Visual Studio의 **Developer PowerShell for VS 2022**에서 다음 명령으로
Release x64 DLL을 빌드합니다.

```powershell
msbuild .\cpp\AIP_DCS.sln /t:Rebuild /p:Configuration=Release /p:Platform=x64 /m:1
```

일반 PowerShell에서 Visual Studio Community 기본 설치 경로를 사용한다면
다음 명령도 사용할 수 있습니다.

```powershell
& "$env:ProgramFiles\Microsoft Visual Studio\2022\Community\MSBuild\Current\Bin\MSBuild.exe" `
    .\cpp\AIP_DCS.sln `
    /t:Rebuild `
    /p:Configuration=Release `
    /p:Platform=x64 `
    /m:1
```

정상적으로 빌드되면 다음 파일이 생성됩니다.

```text
cpp\build\x64\Release\LadyLuck_v2.dll
```

## 대회 실행 폴더에 배치

이 저장소에는 대회에서 제공하는 Python/Unreal 실행 환경이 포함되어 있지
않습니다. 별도로 준비된 `AIP_LIB\DogFightEnv\Release` 폴더에 DLL과 XML만
복사합니다.

```powershell
$releaseDir = "C:\path\to\AIP_LIB\DogFightEnv\Release"

Copy-Item -LiteralPath .\cpp\build\x64\Release\LadyLuck_v2.dll `
    -Destination (Join-Path $releaseDir "LadyLuck_v2.dll") -Force
Copy-Item -LiteralPath .\cpp\config\LadyLuck_v2.xml `
    -Destination (Join-Path $releaseDir "LadyLuck_v2.xml") -Force
```

배치 대상 Python 실행기는 DLL의 중립 진입점 `StepKinematicObservation`을
지원하는 버전을 사용해야 합니다.

## 실행

대회 서버가 실행 중인 상태에서 `DogFightEnv\Release` 폴더로 이동한 뒤
다음 명령을 실행합니다. 아래 IP와 포트는 실제 대회 서버 값으로 바꿉니다.

```powershell
Set-Location "C:\path\to\AIP_LIB\DogFightEnv\Release"

$serverIp = Read-Host "대회 서버 IP"
$serverPort = [int](Read-Host "대회 서버 포트")

python .\run_unreal_inference.py `
    --mode bt `
    --team-name LadyLuck `
    --server-ip $serverIp `
    --server-port $serverPort `
    --ai-type rule `
    --action-repeat 1 `
    --command-delay-sec 0
```

Writer와 명령 상태를 CSV로 기록하려면 다음 인자를 추가합니다.

```powershell
--bt-diagnostics-csv .\Log\LadyLuck_v2_writer_live.csv
```

`LadyLuck_v2.dll`과 `LadyLuck_v2.xml`은 반드시 같은 소스 revision에서
빌드·복사한 조합을 사용해야 합니다.


# 참고문헌 
[1] Bonanni, Pete. "The art of the kill: A comprehensive guide to modern air combat." Spectrum Holo-Byte (1991).<br>
[2] Swihart, Donald E., et al. "Automatic ground collision avoidance system design, integration, & flight test." IEEE Aerospace and Electronic Systems Magazine 26.5 (2011): 4-11.<br>
[3] Yang, Kwangjin, et al. "Manual-based automated maneuvering decisions for air-to-air combat." Journal of Aerospace Information Systems 21.1 (2024): 28-36.<br>
[4] You, Dong-Il, and David Hyunchul Shim. "Design of an aerial combat guidance law using virtual pursuit point concept." Proceedings of the Institution of Mechanical Engineers, Part G: Journal of Aerospace Engineering 229.5 (2015): 792-813.<br>
[5]Shin, Heemin, et al. "An autonomous aerial combat framework for two-on-two engagements based on basic fighter maneuvers." Aerospace Science and Technology 72 (2018): 305-315.<br>
[6] Berndt, Jon. "JSBSim: An open source flight dynamics model in C++." AIAA modeling and simulation technologies conference and exhibit. 2004.
