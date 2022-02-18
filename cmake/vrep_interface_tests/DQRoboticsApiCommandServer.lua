--(C) Copyright 2022 DQ Robotics Developers
--This file is part of DQ Robotics.
--
--   DQ Robotics is free software: you can redistribute it and/or modify
--    it under the terms of the GNU Lesser General Public License as published by
--    the Free Software Foundation, either version 3 of the License, or
--    (at your option) any later version.
--
--    DQ Robotics is distributed in the hope that it will be useful,
--    but WITHOUT ANY WARRANTY; without even the implied warranty of
--    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
--    GNU Lesser General Public License for more details.
--
--    You should have received a copy of the GNU Lesser General Public License
--    along with DQ Robotics.  If not, see <http://www.gnu.org/licenses/>.

--Contributors:
-- Juan Jose Quiroz Omana -  juanjqo@g.ecc.u-tokyo.ac.jp



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
-- * @brief Returns the mass a handle.
-- * @param inInts. The integer value of the handle. Example: {1} 
-- * @param inFloats. The float input value. Example: {} 
-- * @param inStrings. The string input value. Example: {"absolute_frame"} or {}
-- * @param inBuffer. The buffer input value. Example: {}
-- * @returns the mass
function get_mass(inInts,inFloats,inStrings,inBuffer)
    local mass = {}    
    if #inInts>=1 then    
        mass =sim.getShapeMass(inInts[1]) 
        --print(mass)        
        return {},{mass},{},''              
    end
end
----------------------------------------------------------------------------------


----------------------------------------------------------------------------------
--/**
-- * @brief Returns the inertia of a handle.
-- * @param inInts. The integer value of the handle. Example: {1} 
-- * @param inFloats. The float input value. Example: {} 
-- * @param inStrings. The string input value. Example: {"absolute_frame"} or {}
-- * @param inBuffer. The buffer input value. Example: {}
-- * @returns the inertia
function get_inertia(inInts,inFloats,inStrings,inBuffer)
      
    if #inInts>=1 then
        IM, matrixSFCOM=sim.getShapeInertia(inInts[1])         
   
        if inStrings[1] == 'absolute_frame' then
            matrix0_SF=sim.getObjectMatrix(inInts[1],-1) 
            M = sim.multiplyMatrices(matrix0_SF,matrixSFCOM)        
            I= {{IM[1],IM[2],IM[3]},
                {IM[4],IM[5],IM[6]},
                {IM[7],IM[8],IM[9]}}        
            R_0_COM = {{M[1],M[2],M[3]},
                       {M[5],M[6],M[7]},
                       {M[9],M[10],M[11]}}
            R_0_COM_T = transpose(R_0_COM)            
            RIR_T = mat_mult(mat_mult(R_0_COM,I), R_0_COM_T)
            RIR_T_v = {RIR_T[1][1], RIR_T[1][2], RIR_T[1][3],
                       RIR_T[2][1], RIR_T[2][2], RIR_T[2][3],
                       RIR_T[3][1], RIR_T[3][2], RIR_T[3][3]}
            resultInertiaMatrix=RIR_T_v
        else
            resultInertiaMatrix=IM
        end      
        return {},resultInertiaMatrix,{},''              
    end
end
----------------------------------------------------------------------------------


----------------------------------------------------------------------------------
--/**
-- * @brief Returns the center of mass of a handle.
-- * @param inInts. The integer value of the handle. Example: {1} 
-- * @param inFloats. The float input value. Example: {} 
-- * @param inStrings. The string input value. Example: {"absolute_frame"} or {}
-- * @param inBuffer. The buffer input value. Example: {}
-- * @returns the center of mass
function get_center_of_mass(inInts,inFloats,inStrings,inBuffer)
    if #inInts>=1 then
        IM,matrix_SF_COM=sim.getShapeInertia(inInts[1])
        matrix0_SF=sim.getObjectMatrix(inInts[1],-1)       
        if inStrings[1] == 'absolute_frame' then
            resultMatrix = sim.multiplyMatrices(matrix0_SF,matrix_SF_COM)
        else
            resultMatrix = matrix_SF_COM
        end        
        return {},{resultMatrix[4], resultMatrix[8],resultMatrix[12]},{},''              
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
