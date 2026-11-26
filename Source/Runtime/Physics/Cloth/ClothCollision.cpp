#include "EnginePCH.h"
#include "ClothCollision.h"

#include "Components/SkeletalMeshComponent.h"
#include "Physics/PhysicsAsset.h"
#include "Physics/BodyInstance.h"
#include "Physics/PhysScene.h"
#include "Physics/PhysicsFilter.h"
#include "Physics/PhysicsMath.h"
#include "Math/Box.h"
#include "Render/LineBatcher.h"

#include <PxPhysicsAPI.h>
#include <extensions/PxShapeExt.h>
#include <geometry/PxConvexMesh.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <span>

namespace
{
	FVector XYZ(const FVector4& V)
	{
		return FVector(V.X, V.Y, V.Z);
	}

	void DrawClothConvex(FLineBatcher& Lines, const TArray<FVector4>& Planes,
	                     uint32 Mask, const FMatrix& ClothToWorld, const FVector4& Color)
	{
		constexpr float ParallelEpsilon = 1e-6f;
		constexpr float PlaneTolerance = 0.01f;
		constexpr float Infinity = std::numeric_limits<float>::infinity();
		const auto HasPlane = [Mask](int32 Index)
		{
			return (Mask & (uint32{1} << Index)) != 0;
		};

		for (int32 I = 0; I < Planes.Num(); ++I)
		{
			if (!HasPlane(I))
				continue;

			for (int32 J = I + 1; J < Planes.Num(); ++J)
			{
				if (!HasPlane(J))
					continue;

				const FVector A = XYZ(Planes[I]);
				const FVector B = XYZ(Planes[J]);
				const FVector Cross = A.Cross(B);
				const float LengthSquared = Cross.SizeSquared();
				if (LengthSquared <= ParallelEpsilon * ParallelEpsilon)
					continue;

				const FVector Origin =
				    (A * Planes[J].W - B * Planes[I].W).Cross(Cross) / LengthSquared;
				const FVector Direction = Cross / std::sqrt(LengthSquared);
				float Lo = -Infinity;
				float Hi = Infinity;

				// Clip the line shared by these two planes to the other hull half-spaces.
				for (int32 K = 0; K < Planes.Num(); ++K)
				{
					if (K == I || K == J || !HasPlane(K))
						continue;

					const FVector N = XYZ(Planes[K]);
					const float Offset = N.Dot(Origin) + Planes[K].W;
					const float Slope = N.Dot(Direction);
					if (std::abs(Slope) <= ParallelEpsilon)
					{
						if (Offset > PlaneTolerance)
						{
							Lo = Infinity;
							break;
						}
						continue;
					}

					const float T = -Offset / Slope;
					if (Slope > 0.0f)
						Hi = std::min(Hi, T);
					else
						Lo = std::max(Lo, T);
				}

				if (Lo < Hi && std::isfinite(Lo) && std::isfinite(Hi))
				{
					Lines.AddLine(ClothToWorld.TransformPosition(Origin + Direction * Lo),
					              ClothToWorld.TransformPosition(Origin + Direction * Hi), Color);
				}
			}
		}
	}

	bool IsFinite(const FVector& V)
	{
		return std::isfinite(V.X) && std::isfinite(V.Y) && std::isfinite(V.Z);
	}

	bool IsPositiveFinite(float Value)
	{
		return std::isfinite(Value) && Value > 0.0f;
	}

	bool IsClothCollider(const FKShapeElem& Shape)
	{
		return Shape.CollisionEnabled == ECollisionEnabled::PhysicsOnly ||
		       Shape.CollisionEnabled == ECollisionEnabled::QueryAndPhysics;
	}

	bool AppendRoundShape(FClothCollisionBinding& Out, int32 BoneIndex,
	                      const FVector& A, const FVector& B,
	                      float Radius, bool bCapsule)
	{
		const int32 NumSlots = bCapsule ? 2 : 1;
		const int32 First = Out.LocalSpheres.Num();
		if (First + NumSlots > MaxClothSpheres ||
		    !IsPositiveFinite(Radius) || !IsFinite(A) || !IsFinite(B))
		{
			return false;
		}

		Out.LocalSpheres.Add({BoneIndex, FVector4(A.X, A.Y, A.Z, Radius)});
		if (bCapsule)
		{
			Out.LocalSpheres.Add({BoneIndex, FVector4(B.X, B.Y, B.Z, Radius)});
			Out.CapsuleIndices.Add(static_cast<uint32>(First));
			Out.CapsuleIndices.Add(static_cast<uint32>(First + 1));
		}
		return true;
	}

	bool AppendHull(FClothCollisionBinding& Out, int32 BoneIndex,
	                std::span<const FVector4> Planes)
	{
		const int32 First = Out.LocalPlanes.Num();
		if (Planes.size() < 4 ||
		    Planes.size() > static_cast<size_t>(MaxClothPlanes - First))
		{
			return false;
		}

		for (const FVector4& Plane : Planes)
		{
			if (!IsPositiveFinite(XYZ(Plane).Size()) || !std::isfinite(Plane.W))
				return false;
		}

		uint32 Mask = 0;
		for (const FVector4& Plane : Planes)
		{
			Mask |= uint32{1} << Out.LocalPlanes.Num();
			Out.LocalPlanes.Add({BoneIndex, Plane});
		}
		Out.ConvexMasks.Add(Mask);
		return true;
	}

	bool AppendBox(FClothCollisionBinding& Out, int32 BoneIndex,
	               const FKBoxElem& Box)
	{
		if (!IsFinite(Box.Center) || !IsPositiveFinite(Box.X) ||
		    !IsPositiveFinite(Box.Y) || !IsPositiveFinite(Box.Z))
		{
			return false;
		}

		const FQuat Rotation = Box.Rotation.Quaternion();
		const FVector Axes[] = {
		    Rotation.RotateVector(FVector(1.0f, 0.0f, 0.0f)),
		    Rotation.RotateVector(FVector(0.0f, 1.0f, 0.0f)),
		    Rotation.RotateVector(FVector(0.0f, 0.0f, 1.0f))};
		const float HalfExtents[] = {Box.X * 0.5f, Box.Y * 0.5f, Box.Z * 0.5f};

		std::array<FVector4, 6> Planes;
		for (int32 Axis = 0; Axis < 3; ++Axis)
		{
			const FVector& N = Axes[Axis];
			const float CenterProjection = N.Dot(Box.Center);
			Planes[2 * Axis] = FVector4(
			    N.X, N.Y, N.Z, -CenterProjection - HalfExtents[Axis]);
			Planes[2 * Axis + 1] = FVector4(
			    -N.X, -N.Y, -N.Z, CenterProjection - HalfExtents[Axis]);
		}
		return AppendHull(Out, BoneIndex, Planes);
	}

	bool AppendConvex(FClothCollisionBinding& Out, int32 BoneIndex,
	                  USkeletalBodySetup& Setup, const FKConvexElem& Convex,
	                  physx::PxPhysics& Physics)
	{
		FKConvexElem BoneLocal;
		BoneLocal.VertexData.Reserve(Convex.VertexData.Num());

		const FMatrix ShapeToBone = Convex.Transform.GetLocalMatrix();
		for (const FVector& Vertex : Convex.VertexData)
			BoneLocal.VertexData.Add(ShapeToBone.TransformPosition(Vertex));

		const auto Release = [](physx::PxConvexMesh* Mesh)
		{
			Mesh->release();
		};

		const std::unique_ptr<physx::PxConvexMesh, decltype(Release)> Hull(
		    Setup.CreateConvexMesh(BoneLocal, Physics), Release);

		if (!Hull || Hull->getNbPolygons() >
		                 static_cast<uint32>(MaxClothPlanes - Out.LocalPlanes.Num()))
		{
			return false;
		}

		std::array<FVector4, MaxClothPlanes> Planes;
		for (uint32 Index = 0; Index < Hull->getNbPolygons(); ++Index)
		{
			physx::PxHullPolygon Face;
			if (!Hull->getPolygonData(Index, Face))
				return false;

			Planes[Index] = FVector4(
			    Face.mPlane[0], Face.mPlane[1], Face.mPlane[2], Face.mPlane[3]);
		}

		return AppendHull(Out, BoneIndex,
		                  std::span(Planes.data(), Hull->getNbPolygons()));
	}

	physx::PxBounds3 BuildClothSearchBounds(
	    const physx::PxTransform& ClothToWorld,
	    const TArray<FVector>& Skinned,
	    const TArray<float>& MaxDistances,
	    const physx::PxBounds3& CurrentWorldBounds)
	{
		physx::PxBounds3 Bounds = CurrentWorldBounds;
		for (int32 I = 0; I < Skinned.Num(); ++I)
		{
			const physx::PxVec3 P =
			    ClothToWorld.transform(PhysicsMath::ToPx(Skinned[I]));
			const physx::PxVec3 R(MaxDistances[I]);
			Bounds.include(P - R);
			Bounds.include(P + R);
		}
		return Bounds;
	}

	float BoundsDistanceSquared(
	    const physx::PxBounds3& A, const physx::PxBounds3& B)
	{
		const physx::PxVec3 Gap = (A.minimum - B.maximum)
		                              .maximum(B.minimum - A.maximum)
		                              .maximum(physx::PxVec3(0.0f));
		return Gap.magnitudeSquared();
	}

	struct FEnvironmentCandidate
	{
		FClothEnvironmentShapeKey Key;
		float DistanceSquared = 0.0f;
		int32 NumSpheres = 0;
		int32 NumPlanes = 0;
		std::array<FVector4, MaxClothPlanes> Values;
	};

	FVector4 MakeSphere(const physx::PxVec3& Center, float Radius)
	{
		return FVector4(Center.x, Center.y, Center.z, Radius);
	}

	FVector4 MakePlane(const physx::PxVec3& Normal, float D)
	{
		return FVector4(Normal.x, Normal.y, Normal.z, D);
	}

	bool BuildEnvironmentCandidate(
	    const physx::PxGeometryHolder& Geometry,
	    const physx::PxTransform& ShapeToCloth,
	    float CollisionThickness,
	    FEnvironmentCandidate& Out)
	{
		using namespace physx;

		switch (Geometry.getType())
		{
		case PxGeometryType::eSPHERE:
		{
			const float Radius = Geometry.sphere().radius + CollisionThickness;
			if (!IsPositiveFinite(Radius))
				return false;

			Out.Values[0] = MakeSphere(ShapeToCloth.p, Radius);
			Out.NumSpheres = 1;
			return true;
		}

		case PxGeometryType::eCAPSULE:
		{
			const auto& Capsule = Geometry.capsule();
			const float Radius = Capsule.radius + CollisionThickness;
			if (!IsPositiveFinite(Radius))
				return false;

			// PhysX 캡슐의 길이 방향은 Shape 로컬 X축이다.
			const PxVec3 Offset = ShapeToCloth.q.getBasisVector0() * Capsule.halfHeight;
			Out.Values[0] = MakeSphere(ShapeToCloth.p - Offset, Radius);
			Out.Values[1] = MakeSphere(ShapeToCloth.p + Offset, Radius);
			Out.NumSpheres = 2;
			return true;
		}

		case PxGeometryType::eBOX:
		{
			const PxVec3 Half = Geometry.box().halfExtents + PxVec3(CollisionThickness);
			if (!Half.isFinite())
				return false;

			const PxVec3 Axes[] = {
			    ShapeToCloth.q.getBasisVector0(),
			    ShapeToCloth.q.getBasisVector1(),
			    ShapeToCloth.q.getBasisVector2()};

			for (int32 Axis = 0; Axis < 3; ++Axis)
			{
				const PxVec3& N = Axes[Axis];
				const float Projection = N.dot(ShapeToCloth.p);

				Out.Values[2 * Axis] = MakePlane(N, -Projection - Half[Axis]);
				Out.Values[2 * Axis + 1] = MakePlane(-N, Projection - Half[Axis]);
			}

			Out.NumPlanes = 6;
			return true;
		}

		case PxGeometryType::eCONVEXMESH:
		{
			const auto& Convex = Geometry.convexMesh();
			const PxConvexMesh* Hull = Convex.convexMesh;
			const uint32 NumFaces = Hull ? Hull->getNbPolygons() : 0;
			if (NumFaces < 4 || NumFaces > MaxClothPlanes)
				return false;

			const PxMat33 NormalTransform =
			    Convex.scale.getInverse().toMat33().getTranspose();

			for (uint32 I = 0; I < NumFaces; ++I)
			{
				PxHullPolygon Face;
				if (!Hull->getPolygonData(I, Face))
					return false;

				const PxVec3 MeshNormal(Face.mPlane[0], Face.mPlane[1], Face.mPlane[2]);
				PxVec3 N = ShapeToCloth.q.rotate(NormalTransform * MeshNormal);
				const float Length = N.normalize();
				if (!std::isfinite(Length) || Length <= 0.0f || !N.isFinite())
					return false;

				const float D =
				    Face.mPlane[3] / Length -
				    N.dot(ShapeToCloth.p) -
				    CollisionThickness;

				if (!std::isfinite(D))
					return false;

				Out.Values[I] = MakePlane(N, D);
			}

			Out.NumPlanes = static_cast<int32>(NumFaces);
			return true;
		}

		default:
			return false;
		}
	}

	bool TryExpandClothConvexBounds(
	    const physx::PxConvexMeshGeometry& Convex,
	    const physx::PxTransform& ShapeWorld,
	    float CollisionThickness,
	    physx::PxBounds3& Bounds)
	{
		using namespace physx;

		if (CollisionThickness == 0.0f)
			return true;

		const PxConvexMesh* Hull = Convex.convexMesh;
		if (!Hull)
			return false;

		PxReal Mass;
		PxMat33 Inertia;
		PxVec3 MeshCenter;
		Hull->getMassInformation(Mass, Inertia, MeshCenter);

		if (!MeshCenter.isFinite())
			return false;

		const PxMat33 NormalTransform =
		    Convex.scale.getInverse().toMat33().getTranspose();

		float MinFaceDistance = std::numeric_limits<float>::max();
		const PxU32 NumFaces = Hull->getNbPolygons();

		for (PxU32 Index = 0; Index < NumFaces; ++Index)
		{
			PxHullPolygon Face;
			if (!Hull->getPolygonData(Index, Face))
				return false;

			const PxVec3 MeshNormal(
			    Face.mPlane[0], Face.mPlane[1], Face.mPlane[2]);
			const float NormalLength =
			    (NormalTransform * MeshNormal).magnitude();

			if (!IsPositiveFinite(NormalLength))
				return false;

			const float FaceDistance =
			    -(MeshNormal.dot(MeshCenter) + Face.mPlane[3]) / NormalLength;

			if (!IsPositiveFinite(FaceDistance))
				return false;

			MinFaceDistance = std::min(MinFaceDistance, FaceDistance);
		}

		const float BoundsScale = 1.0f + CollisionThickness / MinFaceDistance;
		const PxVec3 WorldCenter = ShapeWorld.transform(
		    Convex.scale.transform(MeshCenter));

		const PxBounds3 ExpandedBounds(
		    WorldCenter + (Bounds.minimum - WorldCenter) * BoundsScale,
		    WorldCenter + (Bounds.maximum - WorldCenter) * BoundsScale);

		if (!ExpandedBounds.isFinite())
			return false;

		Bounds = ExpandedBounds;
		return true;
	}
}

FClothCollisionBinding BuildClothCollisionBinding(
    const USkeletalMeshComponent& Component, const UPhysicsAsset& Asset,
    physx::PxPhysics* Physics)
{
	FClothCollisionBinding Out;

	for (USkeletalBodySetup* Setup : Asset.SkeletalBodySetups)
	{
		if (!Setup)
			continue;

		const int32 Bone = Component.FindBoneIndex(Setup->BoneName.ToString());
		if (Bone < 0)
		{
			KRS_LOG(Warning, "[Cloth] {}: missing bone {} in collision asset.",
			        Component.GetName(), Setup->BoneName.ToString());
			continue;
		}

		for (const auto& Sphere : Setup->AggGeom.SphereElems)
		{
			if (IsClothCollider(Sphere) &&
			    !AppendRoundShape(Out, Bone, Sphere.Center, Sphere.Center,
			                      Sphere.Radius, false))
			{
				++Out.NumSkippedShapes;
			}
		}

		for (const auto& Capsule : Setup->AggGeom.SphylElems)
		{
			if (!IsClothCollider(Capsule))
				continue;

			if (!std::isfinite(Capsule.Length) || Capsule.Length < 0.0f)
			{
				++Out.NumSkippedShapes;
				continue;
			}

			const FVector HalfAxis = Capsule.Rotation.Quaternion().RotateVector(
			    FVector(0.0f, 0.0f, Capsule.Length * 0.5f));

			if (!AppendRoundShape(Out, Bone, Capsule.Center - HalfAxis,
			                      Capsule.Center + HalfAxis, Capsule.Radius,
			                      Capsule.Length > 0.0f))
			{
				++Out.NumSkippedShapes;
			}
		}

		for (const auto& Box : Setup->AggGeom.BoxElems)
		{
			if (IsClothCollider(Box) && !AppendBox(Out, Bone, Box))
				++Out.NumSkippedShapes;
		}

		for (const auto& Convex : Setup->AggGeom.ConvexElems)
		{
			if (IsClothCollider(Convex) &&
			    (!Physics || !AppendConvex(Out, Bone, *Setup, Convex, *Physics)))
			{
				++Out.NumSkippedShapes;
			}
		}
	}

	return Out;
}

bool BuildClothCollisionFrame(
    const FClothCollisionBinding& Binding,
    const TArray<FMatrix>& BoneToComponent,
    float CollisionThickness,
    FClothCollisionFrame& OutFrame)
{
	OutFrame = {};

	for (const auto& Sphere : Binding.LocalSpheres)
	{
		if (!BoneToComponent.IsValidIndex(Sphere.BoneIndex))
			return false;

		const FMatrix& Matrix = BoneToComponent[Sphere.BoneIndex];
		const FVector Center = Matrix.TransformPosition(XYZ(Sphere.Value));
		const float Scale =
		    std::max({Matrix.TransformVector(FVector(1.0f, 0.0f, 0.0f)).Size(),
		              Matrix.TransformVector(FVector(0.0f, 1.0f, 0.0f)).Size(),
		              Matrix.TransformVector(FVector(0.0f, 0.0f, 1.0f)).Size()});
		const float Radius = Sphere.Value.W * Scale;
		const float ExpandedRadius = Radius + CollisionThickness;
		if (!IsFinite(Center) ||
		    !IsPositiveFinite(Radius) ||
		    !IsPositiveFinite(ExpandedRadius))
		{
			return false;
		}

		OutFrame.Spheres.Add(FVector4(Center.X, Center.Y, Center.Z, ExpandedRadius));
	}

	int32 LastBone = INDEX_NONE;
	FMatrix NormalMatrix;
	FVector Translation;

	for (const auto& Plane : Binding.LocalPlanes)
	{
		if (!BoneToComponent.IsValidIndex(Plane.BoneIndex))
			return false;

		if (Plane.BoneIndex != LastBone)
		{
			const FMatrix& Matrix = BoneToComponent[Plane.BoneIndex];
			const float Determinant = Matrix.Determinant();
			if (!std::isfinite(Determinant) || std::abs(Determinant) <= SMALL_NUMBER)
				return false;

			NormalMatrix = Matrix.Inverse().GetTransposed();
			Translation = Matrix.TransformPosition(FVector::ZeroVector);
			LastBone = Plane.BoneIndex;
		}

		const FVector N = NormalMatrix.TransformVector(XYZ(Plane.Value));
		const float Length = N.Size();
		const float D = Plane.Value.W - N.Dot(Translation);
		if (!IsPositiveFinite(Length) || !std::isfinite(D))
			return false;

		const FVector UnitN = N / Length;
		const float PlaneDistance = D / Length - CollisionThickness;

		if (!std::isfinite(PlaneDistance))
			return false;

		OutFrame.Planes.Add(FVector4(UnitN.X, UnitN.Y, UnitN.Z, PlaneDistance));
	}

	OutFrame.CapsuleIndices = Binding.CapsuleIndices;
	OutFrame.ConvexMasks = Binding.ConvexMasks;
	OutFrame.NumBoneSpheres = OutFrame.Spheres.Num();
	OutFrame.NumBoneConvexes = OutFrame.ConvexMasks.Num();

	return true;
}

void AppendClothEnvironmentCollision(
    const FPhysScene& Scene,
    const USkeletalMeshComponent& Component,
    const TArray<FVector>& SkinnedPositions,
    const TArray<float>& MaxDistances,
    const FBox* LastWorldParticleBounds,
    float CollisionThickness,
    FClothCollisionFrame& Frame)
{
	using namespace physx;

	if (!Scene.IsInitialized() || SkinnedPositions.IsEmpty() ||
	    SkinnedPositions.Num() != MaxDistances.Num())
	{
		return;
	}

	if ((Component.GetWorldScale3D() - FVector::OneVector).SizeSquared() > 1e-6f)
		return;

	const PxTransform ClothToWorld(
	    PhysicsMath::ToPx(Component.GetWorldLocation()),
	    PhysicsMath::ToPx(Component.GetWorldRotation().Quaternion().Normalized()));

	if (!ClothToWorld.isValid())
		return;

	for (int32 I = 0; I < SkinnedPositions.Num(); ++I)
	{
		if (!IsFinite(SkinnedPositions[I]) ||
		    !std::isfinite(MaxDistances[I]) || MaxDistances[I] < 0.0f)
		{
			return;
		}
	}

	const PxBounds3 CurrentBounds = LastWorldParticleBounds
	                                    ? PxBounds3(PhysicsMath::ToPx(LastWorldParticleBounds->Min),
	                                                PhysicsMath::ToPx(LastWorldParticleBounds->Max))
	                                    : PxBounds3::empty();

	const PxBounds3 SearchBounds = BuildClothSearchBounds(
	    ClothToWorld, SkinnedPositions, MaxDistances, CurrentBounds);

	if (!SearchBounds.isFinite() || SearchBounds.isEmpty())
		return;

	const PxVec3 RankCenter = LastWorldParticleBounds
	                              ? CurrentBounds.getCenter()
	                              : SearchBounds.getCenter();
	const PxBounds3 RankPoint(RankCenter, RankCenter);

	const ECollisionChannel ClothChannel = Component.GetCollisionObjectType();
	std::vector<FEnvironmentCandidate> Candidates;

	for (const FBodyInstance* Body : Scene.GetBodies())
	{
		if (!Body || !Body->IsInScene())
			continue;

		const auto* Other = Body->GetOwnerComponent();
		if (!Other || !Other->IsRegistered() || Other == &Component ||
		    Other->GetWorld() != Component.GetWorld() ||
		    (Component.GetOwner() && Other->GetOwner() == Component.GetOwner()))
		{
			continue;
		}

		if (Component.GetCollisionResponseToChannel(Body->ObjectType) != ECR_Block)
			continue;

		const PxRigidActor& Actor = *Body->GetActor();

		for (PxU32 ShapeIndex = 0; ShapeIndex < Actor.getNbShapes(); ++ShapeIndex)
		{
			PxShape* Shape = nullptr;
			Actor.getShapes(&Shape, 1, ShapeIndex);
			if (!Shape || !Shape->getFlags().isSet(PxShapeFlag::eSIMULATION_SHAPE) ||
			    PhysicsFilter::GetResponse(
			        Shape->getSimulationFilterData(), ClothChannel) != ECR_Block)
			{
				continue;
			}

			const PxGeometryHolder Geometry = Shape->getGeometry();
			const PxBounds3 ShapeBounds = PxShapeExt::getWorldBounds(*Shape, Actor);
			if (!ShapeBounds.isFinite() || ShapeBounds.isEmpty())
				continue;

			const PxTransform ShapeWorld = PxShapeExt::getGlobalPose(*Shape, Actor);

			PxBounds3 CandidateBounds = ShapeBounds;
			bool bHasCandidateBounds = true;

			switch (Geometry.getType())
			{
			case PxGeometryType::eSPHERE:
			case PxGeometryType::eCAPSULE:
				CandidateBounds.fattenFast(CollisionThickness);
				break;

			case PxGeometryType::eBOX:
				CandidateBounds.fattenFast(std::sqrt(3.0f) * CollisionThickness);
				break;

			case PxGeometryType::eCONVEXMESH:
				bHasCandidateBounds = TryExpandClothConvexBounds(
				    Geometry.convexMesh(), ShapeWorld, CollisionThickness,
				    CandidateBounds);
				break;

			default:
				break;
			}

			if (bHasCandidateBounds &&
				(!CandidateBounds.isFinite() ||
			    !SearchBounds.intersects(CandidateBounds)))
			{
				continue;
			}

			FEnvironmentCandidate Candidate{};
			Candidate.Key = {
			    Other->GetUUID(), Body->InstanceBodyIndex, ShapeIndex,
			    Body->GetCollisionShapeRevision()};
			Candidate.DistanceSquared = BoundsDistanceSquared(RankPoint, ShapeBounds);

			if (BuildEnvironmentCandidate(
			        Shape->getGeometry(),
			        ClothToWorld.transformInv(ShapeWorld),
			        CollisionThickness,
			        Candidate))
			{
				Candidates.push_back(std::move(Candidate));
			}
		}
	}

	std::sort(Candidates.begin(), Candidates.end(),
	          [](const auto& A, const auto& B)
	          {
		          if (A.DistanceSquared != B.DistanceSquared)
			          return A.DistanceSquared < B.DistanceSquared;
		          return A.Key < B.Key;
	          });

	int32 FreeSpheres = MaxClothSpheres - Frame.Spheres.Num();
	int32 FreePlanes = MaxClothPlanes - Frame.Planes.Num();
	std::vector<const FEnvironmentCandidate*> Accepted;

	for (const auto& Candidate : Candidates)
	{
		if (Candidate.NumSpheres > FreeSpheres || Candidate.NumPlanes > FreePlanes)
		{
			++Frame.NumEnvironmentShapesSkippedByBudget;
			continue;
		}

		FreeSpheres -= Candidate.NumSpheres;
		FreePlanes -= Candidate.NumPlanes;
		Accepted.push_back(&Candidate);
	}

	// 선정은 거리순, 솔버 슬롯은 형상 키 순서로 유지한다.
	std::sort(Accepted.begin(), Accepted.end(),
	          [](const auto* A, const auto* B)
	          { return A->Key < B->Key; });

	for (const auto* Shape : Accepted)
	{
		Frame.EnvironmentKeys.push_back(Shape->Key);

		const uint32 FirstSphere = Frame.Spheres.Num();
		for (int32 I = 0; I < Shape->NumSpheres; ++I)
			Frame.Spheres.Add(Shape->Values[I]);

		if (Shape->NumSpheres == 2)
		{
			Frame.CapsuleIndices.Add(FirstSphere);
			Frame.CapsuleIndices.Add(FirstSphere + 1);
		}

		uint32 Mask = 0;
		for (int32 I = 0; I < Shape->NumPlanes; ++I)
		{
			Mask |= uint32{1} << Frame.Planes.Num();
			Frame.Planes.Add(Shape->Values[I]);
		}

		if (Mask != 0)
			Frame.ConvexMasks.Add(Mask);
	}
}

void DrawClothCollisionFrame(FLineBatcher& Lines, const FClothCollisionFrame& Frame,
                             const FMatrix& ClothToWorld)
{
	const FVector4 BoneColor(0.2f, 1.0f, 0.35f, 1.0f);
	const FVector4 EnvironmentColor(0.2f, 0.65f, 1.0f, 1.0f);
	std::array<FVector, MaxClothSpheres> Centers;

	for (int32 Index = 0; Index < Frame.Spheres.Num(); ++Index)
	{
		const FVector4& Sphere = Frame.Spheres[Index];
		Centers[Index] = ClothToWorld.TransformPosition(XYZ(Sphere));
		const FVector4& Color = Index < Frame.NumBoneSpheres ? BoneColor : EnvironmentColor;
		Lines.AddSphere(Centers[Index], Sphere.W, Color);
	}

	for (int32 Index = 0; Index + 1 < Frame.CapsuleIndices.Num(); Index += 2)
	{
		const uint32 A = Frame.CapsuleIndices[Index];
		const uint32 B = Frame.CapsuleIndices[Index + 1];
		if (!Frame.Spheres.IsValidIndex(A) || !Frame.Spheres.IsValidIndex(B))
			continue;

		const FVector Axis = (Centers[B] - Centers[A]).GetSafeNormal();
		const FVector Reference = std::abs(Axis.Z) < 0.9f
		                              ? FVector(0.0f, 0.0f, 1.0f)
		                              : FVector(0.0f, 1.0f, 0.0f);
		const FVector Side = Axis.Cross(Reference).GetSafeNormal();
		const FVector Up = Axis.Cross(Side);
		const FVector4& Color = static_cast<int32>(A) < Frame.NumBoneSpheres ? BoneColor : EnvironmentColor;
		for (const FVector& Direction : {Side, -Side, Up, -Up})
		{
			Lines.AddLine(Centers[A] + Direction * Frame.Spheres[A].W,
			              Centers[B] + Direction * Frame.Spheres[B].W, Color);
		}
	}

	for (int32 Index = 0; Index < Frame.ConvexMasks.Num(); ++Index)
	{
		const FVector4& Color = Index < Frame.NumBoneConvexes ? BoneColor : EnvironmentColor;
		DrawClothConvex(Lines, Frame.Planes, Frame.ConvexMasks[Index], ClothToWorld, Color);
	}
}
