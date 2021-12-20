#ifndef ROBOT_INTERFACE2_NLOHMANNJSONEIGENCONVERSION_HPP
#define ROBOT_INTERFACE2_NLOHMANNJSONEIGENCONVERSION_HPP

#include <type_traits>
#include <iostream>
#include <typeinfo>

#include <Eigen/Core>

#include <nlohmann/json.hpp>

/**
 * Provide to_json() and from_json() overloads for nlohmann::json,
 * which allows simple syntax like:
 *
 * @code
 * Eigen::Matrix3f in, out;
 *
 * json j;
 * j = in;
 * out = j;
 * @endcode
 *
 * @test VirtualRobotJsonEigenConversionTest
 *
 * @see https://github.com/nlohmann/json#arbitrary-types-conversions
 */
namespace Eigen
{

// MatrixBase

template <typename Derived>
void to_json(nlohmann::json& j, const MatrixBase<Derived>& matrix);

template <typename Derived>
void from_json(const nlohmann::json& j, MatrixBase<Derived>& matrix);


// Quaternion

template <typename Derived>
void to_json(nlohmann::json& j, const QuaternionBase<Derived>& quat);

template <typename Derived>
void from_json(const nlohmann::json& j, QuaternionBase<Derived>& quat);



// IMPLEMENTATION

template <typename Derived>
void to_json(nlohmann::json& j, const MatrixBase<Derived>& matrix)
{
    for (int row = 0; row < matrix.rows(); ++row)
    {
        nlohmann::json column = nlohmann::json::array();
        for (int col = 0; col < matrix.cols(); ++col)
        {
            column.push_back(matrix(row, col));
        }
        j.push_back(column);
    }
}

template <typename Derived>
void from_json(const nlohmann::json& j, MatrixBase<Derived>& matrix)
{
    using Scalar = typename MatrixBase<Derived>::Scalar;

    for (std::size_t row = 0; row < j.size(); ++row)
    {
        const auto& jrow = j.at(row);
        for (std::size_t col = 0; col < jrow.size(); ++col)
        {
            const auto& value = jrow.at(col);
            matrix(row, col) = value.get<Scalar>();
        }
    }
}

inline Matrix<double, -1, -1, RowMajor> from_json(const nlohmann::json& j)
{
    Matrix<double, -1, -1, RowMajor> matrix = MatrixXd(j.size(), j.at(0).size());
    for (std::size_t row = 0; row < j.size(); ++row)
    {
        const auto& jrow = j.at(row);
        for (std::size_t col = 0; col < jrow.size(); ++col)
        {
            const auto& value = jrow.at(col);
            matrix(row, col) = value.get<double>();
        }
    }
    return matrix;
}

template <typename Derived>
void to_json(nlohmann::json& j, const QuaternionBase<Derived>& quat)
{
    j["qw"] = quat.w();
    j["qx"] = quat.x();
    j["qy"] = quat.y();
    j["qz"] = quat.z();
}

template <typename Derived>
void from_json(const nlohmann::json& j, QuaternionBase<Derived>& quat)
{
    using Scalar = typename QuaternionBase<Derived>::Scalar;
    quat.w() = j.at("qw").get<Scalar>();
    quat.x() = j.at("qx").get<Scalar>();
    quat.y() = j.at("qy").get<Scalar>();
    quat.z() = j.at("qz").get<Scalar>();
}

}

#endif // ROBOT_INTERFACE2_NLOHMANNJSONEIGENCONVERSION_HPP
