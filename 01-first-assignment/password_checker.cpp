/*
 * 程序读入 "名 姓 密码"，密码需同时满足：
 *   1) 长度至少 10 个字符；
 *   2) 至少含一个大写字母、一个小写字母、一个数字；
 *   3) 不包含使用者的名或姓（区分大小写）。
 * 全部满足输出 VALID，否则输出 INVALID。
 *
 * 原程序的错误（本版已修复）：
 *   1. 数字判断错误：contains_between(password, 0, 9) 传的是整数 0 和 9，
 *      而字符比较按 ASCII 码进行，数字字符 '0'~'9' 的码值是 48~57，
 *      永远落在 (0, 9) 区间之外，导致 digit_ok 恒为 false，
 *      任何密码都无法通过数字检查。应传字符 '0' 和 '9'。
 *   2. 长度判断错误：password.size() <= 10 的含义是"至多 10 个字符"，
 *      与"至少 10 个字符"的要求方向相反，导致长密码被拒、短密码放行。
 *      应为 password.size() >= 10。
 *   3. 范围边界错误：contains_between 内部用严格小于 (lower < ch && ch < upper)，
 *      开区间不含端点，边界字符 'A'、'Z'、'a'、'z'、'0'、'9' 本身不算命中，
 *      例如只含大写字母 'Z' 的密码会被判为"没有大写字母"。应改为 <=。
 *   另外删除了从未使用的 int checked = 0;，消除 -Wall 的未使用变量警告。
 */

#include <iostream>
#include <string>

bool contains_between(const std::string& text, char lower, char upper) {
    for (char ch : text) {
        // 修复 1：原来是 lower < ch && ch < upper（严格小于），
        // 边界字符 'A'、'Z'、'a'、'z'、'0'、'9' 本身不落在开区间内，
        // 例如只含大写字母 'Z' 的密码会被误判为"没有大写字母"。
        // 改为 <= 后，区间两端也计入命中。
        if (lower <= ch && ch <= upper) {
            return true;  
        }
    }
    return false;  
}
bool check_password(const std::string& first_name,
                    const std::string& last_name,
                    const std::string& password) {
    // 修复 2：原来是 password.size() <= 10（至多 10），方向写反了，
    // 会导致超过 10 位的强密码被判 INVALID。
    bool length_ok = password.size() >= 10;


    // 修复 3：原来是 contains_between(password, 0, 9)，
    // 传的是整数 0 和 9；而字符比较按 ASCII 码进行，
    // 数字字符 '0'~'9' 的 ASCII 码是 48~57，永远不在 (0, 9) 内，
    // 导致 digit_ok 恒为 false。必须传字符 '0' 和 '9'。
    bool upper_ok = contains_between(password, 'A', 'Z');
    bool lower_ok = contains_between(password, 'a', 'z');
    bool digit_ok = contains_between(password, '0', '9');

    // find 找不到子串时返回 npos，名和姓都找不到（== npos）才通过。
    // find 本身区分大小写，"john" 不会匹配 "John"。
    bool name_ok = password.find(first_name) == std::string::npos &&
                   password.find(last_name) == std::string::npos;

    // 清理：删除了无用的 int checked = 0;（从未被使用）

    // 所有条件必须同时满足（逻辑与）。
    return length_ok && upper_ok && lower_ok && digit_ok && name_ok;
}

int main() {
    std::string first_name, last_name, password;
    // 输入格式不变：名 姓 密码（以空白分隔）。读取失败则报错退出。
    if (!(std::cin >> first_name >> last_name >> password)) {
        std::cerr << "请输入：名 姓 密码\n";
        return 1;
    }
    // 输出格式不变：全部满足输出 VALID，否则 INVALID。
    std::cout << (check_password(first_name, last_name, password)
                      ? "VALID" : "INVALID") << '\n';
    return 0;
}
