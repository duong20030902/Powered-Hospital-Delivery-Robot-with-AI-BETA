using API_Powered_Hospital_Delivery_Robot.Models.DTOs;
using API_Powered_Hospital_Delivery_Robot.Services.IServices;
using Microsoft.AspNetCore.Http;
using Microsoft.AspNetCore.Mvc;

namespace API_Powered_Hospital_Delivery_Robot.Controllers
{
    [Route("api/[controller]")]
    [ApiController]
    public class UsersController : ControllerBase
    {
        private readonly IUserService _service;

        public UsersController(IUserService service)
        {
            _service = service;
        }

        [HttpGet("get-all")]
        public async Task<ActionResult<IEnumerable<UserResponseDto>>> GetAll([FromQuery] bool? isActive = null)
        {
            var users = await _service.GetAllAsync(isActive);
            return Ok(users);
        }

        [HttpGet("detail/{id}")]
        public async Task<ActionResult<UserResponseDto>> GetById(ulong id)
        {
            var user = await _service.GetByIdAsync(id);
            if (user == null) return NotFound();
            return Ok(user);
        }

        [HttpPost]
        public async Task<ActionResult<UserResponseDto>> Create(UserDto userDto)
        {
            try
            {
                var created = await _service.CreateAsync(userDto);
                return CreatedAtAction(nameof(GetById), new { id = created.Id }, created);
            }
            catch (InvalidOperationException ex)
            {
                return BadRequest(ex.Message);
            }
        }

        [HttpPut("{id}")]
        public async Task<ActionResult<UserResponseDto>> Update(ulong id, UserDto userDto)
        {
            try
            {
                var updated = await _service.UpdateAsync(id, userDto);
                if (updated == null) return NotFound();
                return Ok(updated);
            }
            catch (InvalidOperationException ex)
            {
                return BadRequest(ex.Message);
            }
        }

        [HttpPatch("{id}/activate")]
        public async Task<IActionResult> Activate(ulong id)
        {
            try
            {
                var success = await _service.ToggleActiveAsync(id, true);
                if (!success) return NotFound();
                return Ok($"Tài khoản {id} đã hoạt động");
            }
            catch (InvalidOperationException ex)
            {
                return BadRequest(ex.Message);
            }
        }

        [HttpPatch("{id}/deactivate")]
        public async Task<IActionResult> Deactivate(ulong id)
        {
            try
            {
                var success = await _service.ToggleActiveAsync(id, false);
                if (!success) return NotFound();
                return Ok($"Tài khoản {id} đã bị vô hiệu hóa");
            }
            catch (InvalidOperationException ex)
            {
                return BadRequest(ex.Message);
            }
        }
    }
}
