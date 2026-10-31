#include "VisionBridge.h"
#include "Sockets.h"
#include "SocketSubsystem.h"
#include "Common/TcpSocketBuilder.h"
#include "Dom/JsonObject.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "HAL/FileManager.h"
#include "HAL/PlatformTime.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "HAL/PlatformMath.h"

namespace
{
    void WriteBigEndianU32(uint8* Dst, uint32 V)
    {
        Dst[0] = (V >> 24) & 0xFF;
        Dst[1] = (V >> 16) & 0xFF;
        Dst[2] = (V >> 8) & 0xFF;
        Dst[3] = V & 0xFF;
    }

    uint32 ReadBigEndianU32(const uint8* Src)
    {
        return (uint32(Src[0]) << 24) | (uint32(Src[1]) << 16) | (uint32(Src[2]) << 8) | uint32(Src[3]);
    }
}

FVisionBridge::~FVisionBridge()
{
    CloseSocket();
}

void FVisionBridge::Configure(const FString& Host, int32 Port,
                              EVisionCacheMode InCacheMode, const FString& CacheDirName)
{
    ServerHost = Host;
    ServerPort = Port;
    CacheMode = InCacheMode;
    bLegacyTransfer = FParse::Param(FCommandLine::Get(), TEXT("ThesisLegacyVisionTransfer"));
    CacheDir = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("VisionCache"), CacheDirName);
    bConnectionFailed = false;

    if (CacheMode == EVisionCacheMode::Record)
    {
        IFileManager::Get().MakeDirectory(*CacheDir, true);
    }

    UE_LOG(LogTemp, Log, TEXT("Thesis vision bridge: %s:%d, cache mode %d, cache dir %s"),
           *ServerHost, ServerPort, static_cast<int32>(CacheMode), *CacheDir);
    UE_LOG(LogTemp, Log, TEXT("Thesis vision transfer: %s"),
           bLegacyTransfer ? TEXT("legacy") : TEXT("optimized"));
}

FString FVisionBridge::CachePath(int32 DroneIndex, int32 Tick) const
{
    return FPaths::Combine(CacheDir, FString::Printf(TEXT("d%d_t%06d.json"), DroneIndex, Tick));
}

bool FVisionBridge::ParseDetectionsJson(const FString& Json, TArray<FVisionDetection>& Out, bool bStrictValidation)
{
    Out.Reset();

    TSharedPtr<FJsonObject> Root;
    const TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(Json);
    if (!FJsonSerializer::Deserialize(Reader, Root) || !Root.IsValid())
    {
        return false;
    }

    const TArray<TSharedPtr<FJsonValue>>* Detections = nullptr;
    if (!Root->TryGetArrayField(TEXT("detections"), Detections) || !Detections)
    {
        return false;
    }

    for (const TSharedPtr<FJsonValue>& V : *Detections)
    {
        const TSharedPtr<FJsonObject>* Obj = nullptr;
        if (!V->TryGetObject(Obj) || !Obj->IsValid())
        {
            if (bStrictValidation)
            {
                UE_LOG(LogTemp, Error, TEXT("Thesis vision cache: malformed detection entry."));
                Out.Reset();
                return false;
            }
            continue;
        }
        FVisionDetection D;
        if (bStrictValidation)
        {
            double ClassId = 0.0, Confidence = 0.0, X0 = 0.0, Y0 = 0.0, X1 = 0.0, Y1 = 0.0;
            const bool bFieldsValid =
                (*Obj)->TryGetNumberField(TEXT("cls"), ClassId) &&
                (*Obj)->TryGetNumberField(TEXT("conf"), Confidence) &&
                (*Obj)->TryGetNumberField(TEXT("x0"), X0) &&
                (*Obj)->TryGetNumberField(TEXT("y0"), Y0) &&
                (*Obj)->TryGetNumberField(TEXT("x1"), X1) &&
                (*Obj)->TryGetNumberField(TEXT("y1"), Y1);
            const bool bValuesValid = bFieldsValid &&
                FMath::IsFinite(ClassId) && FMath::IsFinite(Confidence) &&
                FMath::IsFinite(X0) && FMath::IsFinite(Y0) &&
                FMath::IsFinite(X1) && FMath::IsFinite(Y1) &&
                ClassId >= 0.0 && ClassId <= static_cast<double>(MAX_int32) &&
                ClassId == FMath::FloorToDouble(ClassId) &&
                Confidence >= 0.0 && Confidence <= 1.0 &&
                X0 >= 0.0 && X0 <= 1.0 && Y0 >= 0.0 && Y0 <= 1.0 &&
                X1 > X0 && X1 <= 1.0 && Y1 > Y0 && Y1 <= 1.0;
            if (!bValuesValid)
            {
                UE_LOG(LogTemp, Error, TEXT("Thesis vision cache: detection has missing, non-finite, or invalid fields."));
                Out.Reset();
                return false;
            }
            D.ClassId    = static_cast<int32>(ClassId);
            D.Confidence = static_cast<float>(Confidence);
            D.X0 = static_cast<float>(X0);
            D.Y0 = static_cast<float>(Y0);
            D.X1 = static_cast<float>(X1);
            D.Y1 = static_cast<float>(Y1);
        }
        else
        {
            D.ClassId    = static_cast<int32>((*Obj)->GetNumberField(TEXT("cls")));
            D.Confidence = static_cast<float>((*Obj)->GetNumberField(TEXT("conf")));
            D.X0 = static_cast<float>((*Obj)->GetNumberField(TEXT("x0")));
            D.Y0 = static_cast<float>((*Obj)->GetNumberField(TEXT("y0")));
            D.X1 = static_cast<float>((*Obj)->GetNumberField(TEXT("x1")));
            D.Y1 = static_cast<float>((*Obj)->GetNumberField(TEXT("y1")));
        }
        Out.Add(D);
    }
    return true;
}

bool FVisionBridge::EnsureConnected()
{
    if (Socket)
    {
        return true;
    }
    if (bConnectionFailed)
    {
        return false;
    }

    ISocketSubsystem* SocketSubsystem = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM);
    if (!SocketSubsystem)
    {
        bConnectionFailed = true;
        return false;
    }

    TSharedRef<FInternetAddr> Addr = SocketSubsystem->CreateInternetAddr();
    bool bValidIp = false;
    Addr->SetIp(*ServerHost, bValidIp);
    Addr->SetPort(ServerPort);
    if (!bValidIp)
    {
        UE_LOG(LogTemp, Error, TEXT("Thesis vision bridge: invalid host '%s' — vision disabled for this run."), *ServerHost);
        bConnectionFailed = true;
        return false;
    }

    Socket = FTcpSocketBuilder(TEXT("ThesisVisionBridge"))
        .AsBlocking()
        .WithReceiveBufferSize(4 * 1024 * 1024)
        .WithSendBufferSize(4 * 1024 * 1024);

    if (!Socket || !Socket->Connect(*Addr))
    {
        UE_LOG(LogTemp, Error,
               TEXT("Thesis vision bridge: cannot connect to %s:%d — start Tools/vision_server.py first. Vision disabled for this run."),
               *ServerHost, ServerPort);
        CloseSocket();
        bConnectionFailed = true;
        return false;
    }

    Socket->SetNoDelay(true);
    UE_LOG(LogTemp, Log, TEXT("Thesis vision bridge: connected to %s:%d"), *ServerHost, ServerPort);
    return true;
}

void FVisionBridge::CloseSocket()
{
    if (Socket)
    {
        Socket->Close();
        if (ISocketSubsystem* SocketSubsystem = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM))
        {
            SocketSubsystem->DestroySocket(Socket);
        }
        Socket = nullptr;
    }
}

bool FVisionBridge::SendAll(const uint8* Data, int32 Len)
{
    int32 TotalSent = 0;
    while (TotalSent < Len)
    {
        int32 Sent = 0;
        if (!Socket->Send(Data + TotalSent, Len - TotalSent, Sent) || Sent <= 0)
        {
            return false;
        }
        TotalSent += Sent;
    }
    return true;
}

bool FVisionBridge::RecvAll(uint8* Data, int32 Len)
{
    int32 TotalRead = 0;
    while (TotalRead < Len)
    {
        int32 Read = 0;
        if (!Socket->Recv(Data + TotalRead, Len - TotalRead, Read, ESocketReceiveFlags::WaitAll) || Read <= 0)
        {
            return false;
        }
        TotalRead += Read;
    }
    return true;
}

bool FVisionBridge::Detect(int32 DroneIndex, int32 Tick, int32 Width, int32 Height,
                           const TArray<FColor>& PixelsBGRA, TArray<FVisionDetection>& OutDetections)
{
    LastRequestTimings = FVisionRequestTimings{};
    OutDetections.Reset();

    // ---------- Replay: cache only, never the server ----------
    if (CacheMode == EVisionCacheMode::Replay)
    {
        FString Json;
        if (!FFileHelper::LoadFileToString(Json, *CachePath(DroneIndex, Tick)))
        {
            // Record writes empty detections too; a missing entry means an
            // incomplete or incompatible cache, never a valid negative frame.
            UE_LOG(LogTemp, Error, TEXT("Thesis vision: missing replay frame %s."), *CachePath(DroneIndex, Tick));
            return false;
        }
        const bool bStrictRecovery = FParse::Param(FCommandLine::Get(), TEXT("ThesisPersonGeometricRecovery"));
        return ParseDetectionsJson(Json, OutDetections, bStrictRecovery);
    }

    // ---------- Live (Off / Record) ----------
    if (!EnsureConnected())
    {
        return false;
    }
    if (PixelsBGRA.Num() != Width * Height)
    {
        UE_LOG(LogTemp, Warning, TEXT("Thesis vision bridge: pixel count %d != %dx%d"), PixelsBGRA.Num(), Width, Height);
        return false;
    }

    const FString Header = FString::Printf(
        TEXT("{\"type\":\"detect\",\"drone\":%d,\"tick\":%d,\"width\":%d,\"height\":%d,\"format\":\"bgra8\"}"),
        DroneIndex, Tick, Width, Height);
    const FTCHARToUTF8 HeaderUtf8(*Header);

    uint8 LenPrefix[4];
    WriteBigEndianU32(LenPrefix, static_cast<uint32>(HeaderUtf8.Length()));

    const double SendStart = FPlatformTime::Seconds();
    bool bSent = false;
    if (bLegacyTransfer)
    {
        bSent = SendAll(LenPrefix, 4) &&
            SendAll(reinterpret_cast<const uint8*>(HeaderUtf8.Get()), HeaderUtf8.Length()) &&
            SendAll(reinterpret_cast<const uint8*>(PixelsBGRA.GetData()), PixelsBGRA.Num() * sizeof(FColor));
    }
    else
    {
        const int32 HeaderPacketBytes = 4 + HeaderUtf8.Length();
        RequestHeaderBuffer.SetNumUninitialized(HeaderPacketBytes);
        FMemory::Memcpy(RequestHeaderBuffer.GetData(), LenPrefix, 4);
        FMemory::Memcpy(RequestHeaderBuffer.GetData() + 4, HeaderUtf8.Get(), HeaderUtf8.Length());
        bSent = SendAll(RequestHeaderBuffer.GetData(), RequestHeaderBuffer.Num()) &&
            SendAll(reinterpret_cast<const uint8*>(PixelsBGRA.GetData()), PixelsBGRA.Num() * sizeof(FColor));
    }
    LastRequestTimings.SendSeconds = FPlatformTime::Seconds() - SendStart;

    uint8 RespLenBytes[4];
    const double ReceiveStart = FPlatformTime::Seconds();
    if (!bSent || !RecvAll(RespLenBytes, 4))
    {
        LastRequestTimings.ReceiveSeconds = FPlatformTime::Seconds() - ReceiveStart;
        UE_LOG(LogTemp, Error, TEXT("Thesis vision bridge: socket error — vision disabled for the rest of the run."));
        CloseSocket();
        bConnectionFailed = true;
        return false;
    }

    const uint32 RespLen = ReadBigEndianU32(RespLenBytes);
    if (RespLen == 0 || RespLen > 16u * 1024u * 1024u)
    {
        LastRequestTimings.ReceiveSeconds = FPlatformTime::Seconds() - ReceiveStart;
        UE_LOG(LogTemp, Error, TEXT("Thesis vision bridge: bad response length %u — vision disabled."), RespLen);
        CloseSocket();
        bConnectionFailed = true;
        return false;
    }

    TArray<uint8> LegacyResponseBuffer;
    TArray<uint8>& RespBytes = bLegacyTransfer ? LegacyResponseBuffer : ResponseBuffer;
    RespBytes.SetNumUninitialized(static_cast<int32>(RespLen) + 1);
    if (!RecvAll(RespBytes.GetData(), static_cast<int32>(RespLen)))
    {
        LastRequestTimings.ReceiveSeconds = FPlatformTime::Seconds() - ReceiveStart;
        CloseSocket();
        bConnectionFailed = true;
        return false;
    }
    LastRequestTimings.ReceiveSeconds = FPlatformTime::Seconds() - ReceiveStart;
    RespBytes[static_cast<int32>(RespLen)] = 0;

    const double ParseAndCacheStart = FPlatformTime::Seconds();
    const FString Json = FString(UTF8_TO_TCHAR(reinterpret_cast<const char*>(RespBytes.GetData())));
    if (!ParseDetectionsJson(Json, OutDetections, false))
    {
        LastRequestTimings.ParseAndCacheSeconds = FPlatformTime::Seconds() - ParseAndCacheStart;
        UE_LOG(LogTemp, Warning, TEXT("Thesis vision bridge: unparseable response: %s"), *Json.Left(200));
        return false;
    }

    if (CacheMode == EVisionCacheMode::Record)
    {
        if (!FFileHelper::SaveStringToFile(Json, *CachePath(DroneIndex, Tick)))
        {
            LastRequestTimings.ParseAndCacheSeconds = FPlatformTime::Seconds() - ParseAndCacheStart;
            UE_LOG(LogTemp, Error, TEXT("Thesis vision: failed to record frame %s."), *CachePath(DroneIndex, Tick));
            return false;
        }
    }
    LastRequestTimings.ParseAndCacheSeconds = FPlatformTime::Seconds() - ParseAndCacheStart;
    return true;
}
