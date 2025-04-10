-- Get the project root
local project_root = vim.fn.getcwd()
local nvim_path = project_root .. "/.nvim/?.lua"

-- Add the `.nvim` directory in the project root to the Lua module search path
package.path = package.path .. ";" .. nvim_path

return require("init")
