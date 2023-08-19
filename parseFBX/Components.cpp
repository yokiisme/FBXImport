#include "Components.h"
#include "Conversion.h"

namespace
{
    double InchToMm(double value)
    {
        return value * FBXSDK_IN_TO_MM;
    }

    float GetFOVFromHorizontalFOV(FbxCamera& fbxCamera, double fieldOfViewX)
    {
        const double alphaH = fieldOfViewX / 2;
        const double focalLength = InchToMm(fbxCamera.GetApertureWidth()) / 2 / tan(alphaH * kDeg2Rad);
        const double alphaV = atan(InchToMm(fbxCamera.GetApertureHeight()) / 2 / focalLength) * kRad2Deg;
        return static_cast<float>(alphaV * 2);
    }

    float GetFOVFromFocalLength(FbxCamera& fbxCamera, double focalLength)
    {
        // const according to FBX forums ComputeFieldOfView returns vertical FOV when ApertureMode==eVertical, other - horizontal FOV
        const double fieldOfView = fbxCamera.ComputeFieldOfView(focalLength);
        return fbxCamera.GetApertureMode() == FbxCamera::eVertical ? static_cast<float>(fieldOfView) : GetFOVFromHorizontalFOV(fbxCamera, fieldOfView);
    }

    int GateFitFromFBX(FbxPropertyT<FbxCamera::EGateFit> value)
    {
        // convert FbxCamera::EGateFit to Camera::GateFitMode
        switch (value)
        {
        case FbxCamera::EGateFit::eFitNone:
            return 0;
        case FbxCamera::EGateFit::eFitVertical:
            return 1;
        case FbxCamera::EGateFit::eFitHorizontal:
            return 2;
        case FbxCamera::EGateFit::eFitFill:
            return 3;
        case FbxCamera::EGateFit::eFitOverscan:
            return 4;
        case FbxCamera::EGateFit::eFitStretch:
            return 0;
        default:;
        }
        return value.Get() - 1;
    }

    void ConvertCamera(FbxNode* fbxNode, FBXImportCameraComponent& camera)
    {
        FbxCamera* fbxCamera = fbxNode->GetCamera();
        if (fbxCamera)
        {
            camera.fieldOfView = -1;
            camera.gateFitMode = GateFitFromFBX(fbxCamera->GateFit);
            camera.sensorSize.x = InchToMm(fbxCamera->FilmWidth.Get());
            camera.sensorSize.y = InchToMm(fbxCamera->FilmHeight.Get());
            camera.lensShift.x = InchToMm(fbxCamera->FilmOffsetX.Get()) / camera.sensorSize.x;
            camera.lensShift.y = InchToMm(fbxCamera->FilmOffsetY.Get()) / camera.sensorSize.y;
            camera.usePhysicalProperties = fbxCamera->GetApertureMode() == FbxCamera::eFocalLength;
            ConvertFBXDoubleProperty(fbxCamera->FocalLength, camera.focalLength, 0.0f);

            FbxPropertyT<FbxDouble>* property;
            FOVConversionFunction* converter;
            if (GetFieldOfViewProperty(*fbxCamera, property, converter))
            {
                if (IsSet(*property))
                {
                    double value = property->Get();
                    camera.fieldOfView = converter ? converter(*fbxCamera, value) : static_cast<float>(value);
                }
            }

            ConvertFBXDoubleProperty(fbxCamera->NearPlane, camera.nearPlane, -1);
            ConvertFBXDoubleProperty(fbxCamera->FarPlane, camera.farPlane, -1);
            camera.orthographic = fbxCamera->ProjectionType.Get() == FbxCamera::eOrthogonal;

            if (IsSet(fbxCamera->InterestPosition))
            {
                FbxVector4 pos = fbxNode->LclTranslation.Get();
                double dist = (FbxVector4(fbxCamera->InterestPosition.Get()) - pos).Length();
                double size = tan(camera.fieldOfView * kDeg2Rad / 2) * dist;
                camera.orthographicSize = static_cast<float>(size);
            }
            else
                camera.orthographicSize = -1;
        }
    }

    void ConvertLight(FbxNode* fbxNode, FBXImportLightComponent& light)
    {
        FbxLight* fbxLight = fbxNode->GetLight();
        if (fbxLight)
        {
            if (!fbxLight->LightType.IsValid())
            {
                light.type = kLightDirectional;
            }
            else
            {
                switch (fbxLight->LightType.Get())
                {
                case FbxLight::ePoint:          light.type = kLightPoint; break;
                case FbxLight::eDirectional:    light.type = kLightDirectional; break;
                case FbxLight::eSpot:           light.type = kLightSpot; break;
                case FbxLight::eArea:           light.type = kLightRectangle; break;
                default:
                    light.type = kLightDirectional;
                    //Assert("Uknown light type");
                    ReportError("Unknown light type");
                }
            }

            ConvertFBXDoubleProperty(fbxLight->FarAttenuationStart, light.rangeStart, -1);
            ConvertFBXDoubleProperty(fbxLight->FarAttenuationEnd, light.rangeEnd, -1);
            ConvertFBXDoubleProperty(fbxLight->OuterAngle, light.spotAngle, -1);
            if (ConvertFBXDoubleProperty(fbxLight->Intensity, light.intensity, -1))
                light.intensity = ScaleLightIntensity(*fbxLight, light.intensity);
            ConvertFBXColorProperty(fbxLight->Color, light.color, ColorRGBAf(1, 1, 1));
            ConvertFBXBoolProperty(fbxLight->CastShadows, light.castShadows, false);
        }
    }
}

void GetFocalLengthProperty(FbxCamera& fbxCamera, FbxPropertyT<FbxDouble>*& property, FOVConversionFunction*& converter)
{
    property = &fbxCamera.FocalLength;
    converter = GetFOVFromFocalLength;
}

bool GetFieldOfViewProperty(FbxCamera& fbxCamera, FbxPropertyT<FbxDouble>*& property, FOVConversionFunction*& converter)
{
    converter = NULL;
    switch (fbxCamera.GetApertureMode())
    {
    case FbxCamera::eHorizAndVert:
        property = &fbxCamera.FieldOfViewY;
        break;
    case FbxCamera::eHorizontal:
        property = &fbxCamera.FieldOfView;
        converter = GetFOVFromHorizontalFOV;
        break;
    case FbxCamera::eVertical:
        property = &fbxCamera.FieldOfView;
        break;
    case FbxCamera::eFocalLength:
        property = &fbxCamera.FocalLength;
        converter = GetFOVFromFocalLength;
        break;
    default:
        //Assert(!"Unknown Camera AppertureMode");
        ReportError("Unknown Camera AppertureMode");
        return false;
    }

    return true;
}

float ScaleLightIntensity(FbxLight&, float intensity)
{
    return intensity * 0.01f;
}

void ConvertComponents(FbxNode* fbxNode, FBXImportNode& node, FBXImportScene& scene, const FBXImportSettings& settings)
{
    if (fbxNode->GetNodeAttribute() == NULL)
    {
        //printf("NULL Node Attribute\n\n");
    }
    else
    {
        switch (fbxNode->GetNodeAttribute()->GetAttributeType())
        {
            /*case FbxNodeAttribute::eMarker:
                DisplayMarker(pNode);
                break;

            case FbxNodeAttribute::eSkeleton:
                DisplaySkeleton(pNode);
                break;

            case FbxNodeAttribute::eMesh:
                DisplayMesh(pNode);
                break;

            case FbxNodeAttribute::eNurbs:
                DisplayNurb(pNode);
                break;

            case FbxNodeAttribute::ePatch:
                DisplayPatch(pNode);
                break;*/

        case FbxNodeAttribute::eCamera:
        {
            if (settings.importCameras)
            {
                node.rotation *= EulerToQuaternion(Vector3f(.0f, -90.0f, .0f) * kDeg2Rad);
                node.cameraIndex = (int)scene.cameras.size();
                scene.cameras.push_back(FBXImportCameraComponent());
                FBXImportCameraComponent& camera = scene.cameras.back();
                ConvertCamera(fbxNode, camera);
                //ExtractCustomProperties(node, fbxNode->GetNodeAttribute(), settings);
            }
        }
        break;

        case FbxNodeAttribute::eLight:
        {
            if (settings.importLights)
            {
                node.rotation *= EulerToQuaternion(Vector3f(90.0f, 0.0f, .0f) * kDeg2Rad);
                node.lightIndex = (int)scene.lights.size();
                scene.lights.push_back(FBXImportLightComponent());
                FBXImportLightComponent& light = scene.lights.back();
                ConvertLight(fbxNode, light);
                //ExtractCustomProperties(node, fbxNode->GetNodeAttribute(), settings);
            }
        }
        break;

        /*case FbxNodeAttribute::eLodGroup:
            DisplayLodGroup(pNode);
            break;*/
        default:
            break;
        }
    }

    /*DisplayUserProperties(pNode);
    DisplayTarget(pNode);
    DisplayPivotsAndLimits(pNode);
    DisplayTransformPropagation(pNode);
    DisplayGeometricTransform(pNode);

    for(i = 0; i < pNode->GetChildCount(); i++)
    {
        DisplayContent(pNode->GetChild(i));
    }*/
}
