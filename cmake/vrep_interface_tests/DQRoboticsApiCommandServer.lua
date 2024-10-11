-- (C) Copyright 2011-2024 DQ Robotics Developers
-- 
-- This file is part of DQ Robotics.
-- 
--     DQ Robotics is free software: you can redistribute it and/or modify
--     it under the terms of the GNU Lesser General Public License as published by
--     the Free Software Foundation, either version 3 of the License, or
--     (at your option) any later version.
-- 
--     DQ Robotics is distributed in the hope that it will be useful,
--     but WITHOUT ANY WARRANTY; without even the implied warranty of
--     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
--     GNU Lesser General Public License for more details.
-- 
--     You should have received a copy of the GNU Lesser General Public License
--     along with DQ Robotics.  If not, see <http://www.gnu.org/licenses/>.
--
-- DQ Robotics website: dqrobotics.github.io

--Contributors:
-- Juan Jose Quiroz Omana -  juanjqo@g.ecc.u-tokyo.ac.jp
--    - Responsible for the original implementation.
--
-- Frederico Fernandes Afonso Silva - frederico.silva@ieee.org
--    - Generalized functions get_center_of_mass() and get_inertia() to allow arbitrary 
--      reference frames.
--    - Removed unnused inputs (e.g., inFloats, inStrings, and inBuffer) from functions
--      get_mass(), get_center_of_mass(), and get_inertia(). Also renamed some of their
--      variables and added comments to make them clearer.



----------------------------------------------------------------------------------
--/**
-- * @brief This function is used to test the DQ_VrepInterface::call_script_function.
-- * @param inInts. The integer value of the handle. Example: {1} 
-- * @param inFloats. The float input value. Example: {2.0,3.4} 
-- * @param inStrings. The string input value. Example: {"DQ_Robotics"} 
-- * @param inBuffer. The buffer input value. Example: {}
-- * @returns inInts
-- * @returns inFloats
-- * @returns inStrings
-- * @returns Empty
function test_inputs(inInts,inFloats,inStrings,inBuffer)   
    print('----------------')
    print('test_inputs')
    print(inInts)   
    print(inFloats)   
    print(inStrings) 
    print('----------------')   
 
    return inInts,inFloats,inStrings,'' 
end
----------------------------------------------------------------------------------


----------------------------------------------------------------------------------
--/**
-- * @brief Returns the mass of an object.
-- * @param inInts. The integer value of the object's handle. Example: [1]
-- * @returns the object's mass
function get_mass(inInts)
    if #inInts>=1 then
        mass = sim.getShapeMass(inInts[1])
        
        return {}, {mass}, {}, ''
    end
end
----------------------------------------------------------------------------------


----------------------------------------------------------------------------------
--/**
-- * @brief Returns the inertia tensor of an object.
-- * @param inInts. The integer value of the object's handle. Optionally, the
--      handle of the desired reference frame 'df' (the detault is the shape's
--      reference frame 'sf'). Example: [1] or [1, 2]
-- * @param inFloats. The float input value. Example: {} 
-- * @param inStrings. The name of the desired reference frame. Example: {"absolute_frame"} or {}
-- * @returns the inertia tensor of the object
function get_inertia(inInts, inFloats, inStrings)
    if #inInts==1 then
        -- Retrieves the inertia tensor in 'sf'
        I_sf, H_com_sf = sim.getShapeInertia(inInts[1])
        
        if inStrings[1] == 'absolute_frame' then
            -- Retrieves the homogeneous transformation matrix between 'sf' and 'absolute_frame'
            H_sf_0 = sim.getObjectMatrix(inInts[1], -1)

            -- Expresses the center of mass in 'absolute_frame'
            H_com_0 = sim.multiplyMatrices(H_sf_0, H_com_sf)

            -- Formats I_sf for LUA matrix multiplication
            I_lua = {{I_sf[1],I_sf[2],I_sf[3]},
                     {I_sf[4],I_sf[5],I_sf[6]},
                     {I_sf[7],I_sf[8],I_sf[9]}}

            -- Retrieves the rotation matrix from H_com_0
            R_com_0 = {{H_com_0[1],H_com_0[2], H_com_0[3]},
                       {H_com_0[5],H_com_0[6], H_com_0[7]},
                       {H_com_0[9],H_com_0[10],H_com_0[11]}}

            -- Expresses the inertia tensor in 'absolute_frame' (R*I*R^T)
            I_0_lua = mat_mult(mat_mult(R_com_0, I_lua), transpose(R_com_0))

            -- Formats I_0_lua for proper return (as a 1x9 vector)
            I_0 = {I_0_lua[1][1], I_0_lua[1][2], I_0_lua[1][3],
                   I_0_lua[2][1], I_0_lua[2][2], I_0_lua[2][3],
                   I_0_lua[3][1], I_0_lua[3][2], I_0_lua[3][3]}
            
            return {}, I_0, {}, ''
        else
            return {}, I_sf, {}, ''
        end
    elseif #inInts==2 then
        -- Retrieves the inertia tensor and the center of mass in 'sf'
        I_sf, H_com_sf = sim.getShapeInertia(inInts[1])
        
        -- Retrieves the homogeneous transformation matrix between 'sf' and 'df'
        H_sf_df = sim.getObjectMatrix(inInts[1], inInts[2])
        
        -- Expresses the center of mass in 'df'
        H_com_df = sim.multiplyMatrices(H_sf_df, H_com_sf)
        
        -- Formats I_sf for LUA matrix multiplication
        I_lua = {{I_sf[1],I_sf[2],I_sf[3]},
                 {I_sf[4],I_sf[5],I_sf[6]},
                 {I_sf[7],I_sf[8],I_sf[9]}}
        
        -- Retrieves the rotation matrix from H_com_df
        R_com_df = {{H_com_df[1],H_com_df[2], H_com_df[3]},
                    {H_com_df[5],H_com_df[6], H_com_df[7]},
                    {H_com_df[9],H_com_df[10],H_com_df[11]}}
         
        -- Expresses the inertia tensor in 'df' (R*I*R^T)
        I_df_lua = mat_mult(mat_mult(R_com_df, I_lua), transpose(R_com_df))
        
        -- Formats I_df_lua for proper return (as a 1x9 vector)
        I_df = {I_df_lua[1][1], I_df_lua[1][2], I_df_lua[1][3],
                I_df_lua[2][1], I_df_lua[2][2], I_df_lua[2][3],
                I_df_lua[3][1], I_df_lua[3][2], I_df_lua[3][3]}

        return {}, I_df, {}, ''
    end
end
----------------------------------------------------------------------------------


----------------------------------------------------------------------------------
--/**
-- * @brief Returns the center of mass of an object.
-- * @param inInts. The integer value of the object's handle. Optionally, the
--      handle of the desired reference frame 'df' (the detault is the shape's
--      reference frame 'sf'). Example: [1] or [1, 2]
-- * @param inFloats. The float input value. Example: {} 
-- * @param inStrings. The name of the desired reference frame. Example: {"absolute_frame"} or {}
-- * @returns the vector of the object's center of mass
function get_center_of_mass(inInts, inFloats, inStrings)
    if #inInts==1 then
        -- Retrieves the center of mass in 'sf'
        _, H_com_sf = sim.getShapeInertia(inInts[1])
     
        if inStrings[1] == 'absolute_frame' then
            -- Retrieves the homogeneous transformation matrix between 'sf' and 'absolute_frame'
            H_sf_0 = sim.getObjectMatrix(inInts[1], -1)

            -- Expresses the center of mass in 'absolute_frame'
            H_com_0 = sim.multiplyMatrices(H_sf_0, H_com_sf)
            
            return {}, {H_com_0[4], H_com_0[8],H_com_0[12]}, {}, ''
        else
            return {}, {H_com_sf[4], H_com_sf[8],H_com_sf[12]}, {}, ''
        end        
    elseif #inInts==2 then
        -- Retrieves the center of mass in 'sf'
        _, H_com_sf = sim.getShapeInertia(inInts[1])
        
        -- Retrieves the homogeneous transformation matrix between 'sf' and 'df'
        H_sf_df = sim.getObjectMatrix(inInts[1], inInts[2])
        
        -- Expresses the center of mass in 'df'
        H_com_df = sim.multiplyMatrices(H_sf_df, H_com_sf)
        
        return {}, {H_com_df[4], H_com_df[8], H_com_df[12]}, {}, ''
    end
end
----------------------------------------------------------------------------------


----------------------------------------------------------------------------------
--/**
-- * @brief Returns the tranpose matrix of A.
-- * @param Matrix A. Example A = {{1,2,3},{4,5,6},{7,8,9}} 
-- * @returns the tranpose matrix. Given A, tranpose(A) = {{1,4,7},{2,5,8},{3,6,9}} 
function transpose( A )
    local result = {} 
    for i = 1, #A[1] do
        result[i] = {}
        for j = 1, #A do
            result[i][j] = A[j][i]
        end
    end 
    return result
end
----------------------------------------------------------------------------------

----------------------------------------------------------------------------------
--/**
-- * @brief Returns the multiplication between two matrices.
-- * @param Matrix A. Example A = {{1,2,3},{4,5,6},{7,8,9}} 
-- * @param Matrix B. Example B = {{1,2,3},{4,5,6},{7,8,9}} 
-- * @returns the multiplication between A and B. Example = mat_mult(A,B)
function mat_mult( A, B )    
    if #A[1] ~= #B then       -- inner matrix-dimensions must agree        
        return nil      
    end  
    local result = {} 
    for i = 1, #A do
        result[i] = {}
        for j = 1, #B[1] do
            result[i][j] = 0
            for k = 1, #B do
                result[i][j] = result[i][j] + A[i][k] * B[k][j]
            end
        end
    end 
    return result
end
----------------------------------------------------------------------------------
