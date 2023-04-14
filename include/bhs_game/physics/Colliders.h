//
// Created by zartris on 4/14/23.
//

#ifndef BHS_GAME_COLLIDERS_H
#define BHS_GAME_COLLIDERS_H

# include "bhs_game/utils/tensor_alias.h"
# include "torch/torch.h"
# include "bhs_game/utils/global_device.h"

struct CollisionPoint {
    Tensor2Double A; // Furthest point of A into B
    Tensor2Double B; // Furthest point of B into A
    Tensor2Double Normal; // B - A
    Tensor2Double Depth;
    bool HasCollision;
};

struct SphereCollider;
struct BoxCollider;

struct Collider {
    virtual CollisionPoint checkCollision(
            const Tensor3Double *transform,
            const SphereCollider *other,
            const Tensor3Double *otherTransform) const = 0;

    virtual CollisionPoint checkCollision(
            const Tensor3Double *transform,
            const BoxCollider *other,
            const Tensor3Double *otherTransform) const = 0;
};

struct SphereCollider : Collider {
    Tensor3Double Center;
    TScalarDouble Radius;

    CollisionPoint checkCollision(
            const Tensor3Double *transform,
            const SphereCollider *other,
            const Tensor3Double *otherTransform) const override;

    CollisionPoint checkCollision(
            const Tensor3Double *transform,
            const BoxCollider *other,
            const Tensor3Double *otherTransform) const override;

};

struct BoxCollider : Collider {
    Tensor3Double Center; //
    TScalarDouble width; // Distance from center to edge
    TScalarDouble height; // Distance from center to edge

    CollisionPoint checkCollision(
            const Tensor3Double *transform,
            const SphereCollider *other,
            const Tensor3Double *otherTransform) const override;

    CollisionPoint checkCollision(
            const Tensor3Double *transform,
            const BoxCollider *other,
            const Tensor3Double *otherTransform) const override;

};

namespace algo {
    CollisionPoint checkCollision(
            const SphereCollider *a,
            const Tensor3Double *aTransform,
            const SphereCollider *b,
            const Tensor3Double *bTransform);

    CollisionPoint checkCollision(
            const SphereCollider *a,
            const Tensor3Double *aTransform,
            const BoxCollider *b,
            const Tensor3Double *bTransform);

    CollisionPoint checkCollision(
            const BoxCollider *a,
            const Tensor3Double *aTransform,
            const SphereCollider *b,
            const Tensor3Double *bTransform);

    CollisionPoint checkCollision(
            const BoxCollider *a,
            const Tensor3Double *aTransform,
            const BoxCollider *b,
            const Tensor3Double *bTransform);
}


#endif //BHS_GAME_COLLIDERS_H
