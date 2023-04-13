#ifndef MCM_SSP_HPP_GGBE8KES
#define MCM_SSP_HPP_GGBE8KES

#include <vanetza/common/byte_buffer.hpp>
#include <cstdint>
#include <set>
#include <string>

namespace vanetza
{
namespace security
{

/**
 * MCM permission bits
 */
enum class McmPermission : uint16_t
{
    // first octet
    Agreement_Seeking = 0x80,
    Prescriptive = 0x40,

    // second octet
    Interception_Level = 0x8000,
    Road_Safety_Level = 0x4000,
    Global_Traffic_Management_Level = 0x2000,
    Local_Traffic_Management_Level = 0x1000,
    // others are reserved for future usage
};

/**
 * Set of MCM permissions, i.e. Service Specific Permissions
 */
class McmPermissions
{
public:
    McmPermissions();
    McmPermissions(McmPermission);
    McmPermissions(const std::initializer_list<McmPermission>&);

    // check for permission
    bool has(McmPermission) const;
    bool has(const std::initializer_list<McmPermission>&) const;
    bool has(const McmPermissions&) const;
    bool none() const;

    /**
     * Get set of all included permissions
     * \return permission set
     */
    std::set<McmPermission> permissions() const;

    // permission manipulation (with chaining)
    McmPermissions& add(McmPermission);
    McmPermissions& remove(McmPermission);

    // serialization helpers
    ByteBuffer encode() const;
    static McmPermissions decode(const ByteBuffer&);

private:
    using value_type = std::underlying_type<McmPermission>::type;
    value_type m_bits;
};

/**
 * Get string representation of MCM permission flag
 * \param cp MCM permission flag
 * \return string representation (empty for unknown flags)
 */
std::string stringify(McmPermission);

} // namespace security
} // namespace vanetza

#endif /* MCM_SSP_HPP_GGBE8KES */

