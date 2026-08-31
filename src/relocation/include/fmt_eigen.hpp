#include <fmt/format.h>
#include <Eigen/Core>
#include <sstream>

// 为所有 Eigen::Matrix 类型特化 formatter
template <typename Scalar, int Rows, int Cols, int Options, int MaxRows, int MaxCols>
struct fmt::formatter<Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>> {

    // 简单实现：不接受任何格式说明符（如 {:...} 会报错）
    constexpr auto parse(format_parse_context& ctx) {
        // 只允许空的格式说明符 "{}"
        auto it = ctx.begin();
        if (it != ctx.end() && *it != '}')
            throw format_error("Eigen::Matrix formatter does not support format specs");
        return it;
    }

    // 格式化函数
    template <typename FormatContext>
    auto format(const Eigen::Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>& mat,
                FormatContext& ctx) const {
        std::ostringstream oss;
        oss << mat;   // 利用 Eigen 的 operator<<
        return fmt::format_to(ctx.out(), "{}", oss.str());
    }
};