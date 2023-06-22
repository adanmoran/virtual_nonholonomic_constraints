%% Convert Piecewise Symbolic to Function Handle
% This function takes in a piecewise function for a Hamiltonian system,
% with the symbolic variables {p,q}, and converts it to the function handle
% @(q,p) f(q,p). The piecewise function must be well-defined over the
% entire phase space.
%
% Suppose you have the piecewise function 
% f(q,p) = {q^2 + p^2 if q > 0 and p > 0; 0 otherwise}
% Then calling pw2fun(f) will generate a function handle
% @(q,p) (q > 0 && p > 0)*(q^2+p^2) + (1 - (q > 0 && p > 0))*(0)
%
% Inputs:
% * pw = piecewise function with 2 symbolic variables. When symvar is
% called, the variable representing conjugate of momenta must appear first.
% It is assumed that on each region of the piecewise function, the function
% is well-defined (i.e. the function cannot take on the value Inf or NaN).
%
% Outputs:
% * fpw = the piecewise function converted to a function handle
function [fpw] = pw2fun(pw)
    % Get the children of the piecewise function, which corresponds to the
    % function evaluation and the regions for each element of the vector.
    % This may be a cell array or a single entity.
    pwChildren = children(pw);
    % If it is not a cell array, make it into one so we can use the same
    % code for both cases.
    if ~iscell(pwChildren)
        cells = cell(1);
        cells{1} = pwChildren;
        pwChildren = cells;
    end
    % Iterate through every cell in the array to decompose it.
    for i = 1:length(pwChildren)
        % The child has 2 columns, one corresponding to the function and
        % another corresponding to the region it belongs to
        child = pwChildren{i};
        pwFunctions = child(:,1);
        pwRegions = child(:,2);

        % Go through the regions of the function, and replace any value of
        % TRUTH with (1 - product_of_all_other_regions). We do this to
        % enable the use of the dot product, since you can't multiply TRUTH
        % with anything else. We do (1 - r1*...*ri) rather than "not(all
        % other regions)" since "not(all other regions) or (any other
        % region)" is not always true. The TRUTH region is often at the
        % end, so we start at the end and work our way back.
        for r = length(pwRegions):-1:length(pwRegions)
            % If the region is "TRUE", then it is equivalent to "if the
            % input is in no other region".
            isDefaultRegion = double(pwRegions(r) == true);
            % If this is the default region, then we need to set a
            % statement that is "(1 - (all other regions are true))".
            if isDefaultRegion
                % Multiply all regions except this one, subtract it from 1,
                % and that becomes our new region.
                %% TODO: This only works for "x in Dom::" and not if there is a single inequality eg x < 0.
                % We need to convert these to logic statements, as right
                % now they are not being treated as logical yes/no
                % conditions. It might require switching this from symbolic
                % to function handles.
                pwRegions(r) = 1 - ...
                               (prod(pwRegions(cat(2,1:(r-1),(r+1):end))));
                % Now exit the loop, since there is only one TRUTH region
                % per piecewise function.
                break;
            end
        end

        % Now that all the regions are valid symbolic functions, we simply
        % take the dot product of our function values with the regions
        symbolicFpw = dot(pwFunctions, pwRegions);
        
        % Taking the children of a symbolic sum gives the individual sum
        % terms in an array. %% TODO: This is not necessarily true - if you
        % multiply terms it also splits the multipplication :( 
        summationTerms = children(symbolicFpw);
        
        % Now we can apply the safety function to each one and sum them
        % together to get the final version of the piecewise function
        sumFunc = sym2fun(summationTerms);
        fpw = @(q,p) sum(arrayfun(@(x) safety(x), sumFunc(q,p)));
        %% TODO: Get this to work
        % efficiently so we get as explicit a function handle as possible
        % (with little to no recursion). %%
        % We need to get a function handle which applies sum_i
        % safety(summationTerms(i)(q,p)).

    end
end


% Generate a safe value in case the piecewise sections of the piecewise
% function are not defined everywhere. This prevents the undefined
% functions from adding Inf or NaN in the final overall function.
function safeVal = safety(val)
    if isnan(val) || isinf(val)
        safeVal = 0;
    else
        safeVal = val;
    end
end
