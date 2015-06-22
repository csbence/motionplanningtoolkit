#pragma once

#include <random>
#include <fcl/math/transform.h>

template<typename Workspace>
class Transformations {

public:
    typedef fcl::Transform3f Transformation;

    Transformations(const Workspace workspace) : workspace(workspace) {
        auto bounds = workspace.getBounds();
        for (auto range : bounds) {
            distributions.emplace_back(range.first, range.second);
        }
    }

    /**
     * Generate a random transformation within the bounds.
     */
    Transformation sampleTransformation() const {
        fcl::Vec3f translation(distributions[0](generator), distributions[1](generator), distributions[2](generator));

        return fcl::Transform3f(translation);
    };

    /**
     * Length of the given transformation.
     */
    double length(Transformation lhs, Transformation rhs) const {
        return lhs.transform(rhs.getTranslation()).length();
    }

    /**
     * Extract a 3D vector from the given transformation.
     */
    std::vector<double> getVector(Transformation const transformation) const {
        const auto translation = transformation.getTranslation();

        std::vector<double> distanceVector;

        distanceVector.push_back(translation[0]);
        distanceVector.push_back(translation[1]);
        distanceVector.push_back(translation[2]);

        return distanceVector;
    }

private:
    const Workspace workspace;
    std::vector<std::uniform_real_distribution<double> > distributions;
    mutable std::default_random_engine generator;
};