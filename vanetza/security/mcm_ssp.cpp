#include <vanetza/security/mcm_ssp.hpp>
#include <boost/format.hpp>

namespace vanetza
{
namespace security
{

McmPermissions::McmPermissions() : m_bits(0)
{
}

McmPermissions::McmPermissions(McmPermission cp) : m_bits(static_cast<value_type>(cp))
{
}

McmPermissions::McmPermissions(const std::initializer_list<McmPermission>& cps) : m_bits(0)
{
    for (McmPermission cp : cps) {
        add(cp);
    }
}

bool McmPermissions::has(McmPermission cp) const
{
    const auto cp_raw = static_cast<value_type>(cp);
    return (m_bits & cp_raw) == cp_raw;
}

bool McmPermissions::has(const std::initializer_list<McmPermission>& cps) const
{
    for (McmPermission cp : cps) {
        if (!has(cp)) return false;
    }
    return true;
}

bool McmPermissions::has(const McmPermissions& required) const
{
    return (m_bits & required.m_bits) == required.m_bits;
}

bool McmPermissions::none() const
{
    return m_bits == 0;
}

std::set<McmPermission> McmPermissions::permissions() const
{
    std::set<McmPermission> perms;

    std::underlying_type<McmPermission>::type bit = 1;
    for (unsigned i = 1; i < sizeof(bit) * 8; ++i) {
        McmPermission permission = static_cast<McmPermission>(bit);
        if (has(permission)) {
            perms.insert(permission);
        }
        bit <<= 1;
    }

    return perms;
}

McmPermissions& McmPermissions::add(McmPermission cp)
{
    m_bits |= static_cast<value_type>(cp);
    return *this;
}

McmPermissions& McmPermissions::remove(McmPermission cp)
{
    m_bits &= ~static_cast<value_type>(cp);
    return *this;
}

ByteBuffer McmPermissions::encode() const
{
    return ByteBuffer({1, static_cast<uint8_t>(m_bits), static_cast<uint8_t>(m_bits >> 8) });
}

McmPermissions McmPermissions::decode(const ByteBuffer& buffer) // confirm
{
    McmPermissions permissions;
    if (buffer.size() == 3 && buffer[0] == 1) {
        permissions.m_bits = buffer[2];
        permissions.m_bits <<= 8;
        permissions.m_bits |= buffer[1];
    }
    return permissions;
}

std::string stringify(McmPermission permission)
{
    std::string result;
    switch (permission) {
        case McmPermission::Agreement_Seeking:
            result = "Agreement Seeking";
            break;
        case McmPermission::Prescriptive:
            result = "Prescriptive";
            break;
        case McmPermission::Interception_Level:
            result = "Interception level";
            break;
        case McmPermission::Road_Safety_Level:
            result = "Road Safety level";
            break;
        case McmPermission::Global_Traffic_Management_Level:
            result = "Global Traffic Management level";
            break;
        case McmPermission::Local_Traffic_Management_Level:
            result = "Local Traffic Management level";
            break;
        default:
            static_assert(sizeof(McmPermission) == 2, "Expected McmPermission to be 2 bytes wide");
            result = str(boost::format("Reserved (%0#6x)") % static_cast<uint16_t>(permission));
            break;
    }
    return result;
}

} // namespace security
} // namespace vanetza
