//
// Created by zartris on 4/14/23.
//

#include "bhs_game/physics/Colliders.h"

CollisionPoint SphereCollider::checkCollision(const Tensor3Double *transform, const SphereCollider *other,
                                              const Tensor3Double *otherTransform) const {

    return algo::checkCollision(this, transform, other, otherTransform);
}

CollisionPoint SphereCollider::checkCollision(const Tensor3Double *transform, const BoxCollider *other,
                                              const Tensor3Double *otherTransform) const {
    return algo::checkCollision(this, transform, other, otherTransform);
}

CollisionPoint BoxCollider::checkCollision(const Tensor3Double *transform, const SphereCollider *other,
                                           const Tensor3Double *otherTransform) const {
    return algo::checkCollision(this, transform, other, otherTransform);
}

CollisionPoint BoxCollider::checkCollision(const Tensor3Double *transform, const BoxCollider *other,
                                           const Tensor3Double *otherTransform) const {
    return algo::checkCollision(this, transform, other, otherTransform);
}

CollisionPoint algo::checkCollision(const SphereCollider *a, const Tensor3Double *aTransform,
                                    const SphereCollider *b, const Tensor3Double *bTransform) {
    return CollisionPoint();
}

CollisionPoint algo::checkCollision(const SphereCollider *a, const Tensor3Double *aTransform,
                                    const BoxCollider *b, const Tensor3Double *bTransform) {
    return CollisionPoint();
}

CollisionPoint algo::checkCollision(const BoxCollider *a, const Tensor3Double *aTransform,
                                    const SphereCollider *b, const Tensor3Double *bTransform) {
    return CollisionPoint();
}

CollisionPoint algo::checkCollision(const BoxCollider *a, const Tensor3Double *aTransform,
                                    const BoxCollider *b, const Tensor3Double *bTransform) {
    return CollisionPoint();
}