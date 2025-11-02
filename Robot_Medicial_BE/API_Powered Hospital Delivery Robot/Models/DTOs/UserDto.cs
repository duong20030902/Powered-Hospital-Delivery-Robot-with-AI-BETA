using System.ComponentModel.DataAnnotations;

namespace API_Powered_Hospital_Delivery_Robot.Models.DTOs
{
    public class UserDto
    {
        public ulong Id { get; set; }
        public string Username { get; set; } = null!;
        public string Password { get; set; }
        public string FullName { get; set; }
        public string Role { get; set; } 
        public bool IsActive { get; set; }
    }
}
