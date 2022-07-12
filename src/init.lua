local Spline = require(script.Spline)

local DEFAULT_ALPHA = 0.5
local DEFAULT_TENSION = 0
local DEFAULT_PRECOMPUTE_INTERVALS = 16
local EPSILON = 1e-4

type Point = CFrame | Vector2 | Vector3

local CatRom = {}
CatRom.__index = CatRom

local function FuzzyEq(a, b)
	local aType = typeof(a)

	if aType == "number" then
		return a == b or math.abs(a - b) <= (math.abs(a) + 1) * EPSILON
	elseif aType == "Vector3" then
		return a:FuzzyEq(b, EPSILON)
	elseif aType == "Vector2" then
		local aX, bX = a.X, b.X
		local aY, bY = a.Y, b.Y
		return  aX == bX or math.abs(aX - bX) <= (math.abs(aX) + 1) * EPSILON
			and aY == bY or math.abs(aY - bY) <= (math.abs(aY) + 1) * EPSILON
	elseif aType == "CFrame" then
		return a.Position:FuzzyEq(b.Position, EPSILON)
			and a.RightVector:FuzzyEq(b.RightVector, EPSILON)
			and a.UpVector:FuzzyEq(b.UpVector, EPSILON)
			and a.LookVector:FuzzyEq(b.LookVector, EPSILON)
	end

	return false
end

local function CFrameToQuaternion(cframe)
	local axis, angle = cframe:ToAxisAngle()
	angle /= 2
	axis = math.sin(angle) * axis
	return {math.cos(angle), axis.X, axis.Y, axis.Z}
end

local function ToTransform(point, pointType)
	if pointType == "Vector2" or pointType == "Vector3" then
		return {point}
	elseif pointType == "CFrame" then
		return {point.Position, CFrameToQuaternion(point)}
	end

	return nil
end

function CatRom.new(points: {Point}, alpha: number?, tension: number?)
	alpha = alpha or DEFAULT_ALPHA -- Parameterization exponent
	tension = tension or DEFAULT_TENSION

	-- Type check
	assert(type(points) == "table", "Points must be a table")
	assert(type(alpha) == "number", "Alpha must be a number")
	assert(type(tension) == "number", "Tension must be a number")
	assert(#points > 0, "Points table cannot be empty")
	local pointType = typeof(points[1])
	assert(pointType == "Vector2" or pointType == "Vector3" or pointType == "CFrame",
		"Points must be a table of Vector2s, Vector3s, or CFrames")
	for _, point in ipairs(points) do
		assert(typeof(point) == pointType, "All points must have the same type")
	end

	-- Remove equal adjacent points
	local uniquePoints = {} do
		local prevPoint = points[1]
		uniquePoints[1] = prevPoint
		local i = 2
		for j = 2, #points do
			local point = points[j]
			if not FuzzyEq(point, prevPoint) then
				uniquePoints[i] = point
				i += 1
				prevPoint = point
			end
		end
	end
	points = uniquePoints
	local numPoints = #points

	-- Early exit: 1 point
	if numPoints == 1 then
		return setmetatable({
			alpha = alpha,
			tension = tension,

			splines = {Spline.fromPoint(ToTransform(points[1], pointType))},
			domains = {0},

			length = 0
		}, CatRom)
	end

	-- Extrapolate to get 0th and n+1th points
	local firstPoint = points[1]
	local lastPoint = points[numPoints]
	local zerothPoint, veryLastPoint do
		if FuzzyEq(firstPoint, lastPoint) then
			-- Loops
			zerothPoint = points[numPoints - 1]
			veryLastPoint = points[2]
		else
			-- Does not loop
			zerothPoint = points[2]:Lerp(firstPoint, 2)
			veryLastPoint = points[numPoints - 1]:Lerp(lastPoint, 2)
		end
	end

	-- Early exit: 2 points
	if numPoints == 2 then
		local spline = Spline.fromLine(
			ToTransform(zerothPoint, pointType),
			ToTransform(firstPoint, pointType),
			ToTransform(lastPoint, pointType),
			ToTransform(veryLastPoint, pointType)
		)

		return setmetatable({
			alpha = alpha,
			tension = tension,

			splines = {spline},
			domains = {0},

			length = spline.length
		}, CatRom)
	end

	-- Create splines
	local numSplines = numPoints - 1
	local splines = table.create(numSplines)
	local totalLength = 0

	-- Sliding window of control points to quicken instantiating splines
	local window1 = ToTransform(zerothPoint, pointType) -- FIX: Don't pack them in tables instead?
	local window2 = ToTransform(firstPoint, pointType)
	local window3 = ToTransform(points[2], pointType)
	local window4 = ToTransform(points[3] or veryLastPoint, pointType)

	-- Add the first spline, whose first point is not in the points table.
	splines[1] = Spline.new(window1, window2, window3, window4, alpha, tension)
	totalLength += splines[1].length

	-- Add the middle splines
	for i = 1, numPoints - 3 do
		-- Shift the window
		window1 = window2
		window2 = window3
		window3 = window4
		window4 = ToTransform(points[i + 3], pointType)

		local spline = Spline.new(window1, window2, window3, window4, alpha, tension)
		totalLength += spline.length
		splines[i + 1] = spline
	end

	-- Add the last spline, whose fourth point is not in the points table
	splines[numSplines] = Spline.new(window2, window3, window4,
		ToTransform(veryLastPoint, pointType), alpha, tension)
	totalLength += splines[numSplines].length

	-- Get the start of the domain interval for each spline
	local domains = table.create(numSplines - 1)
	local runningLength = 0
	for i, spline in ipairs(splines) do
		domains[i] = runningLength / totalLength
		runningLength += spline.length
	end

	return setmetatable({
		alpha = alpha,
		tension = tension,

		splines = splines,
		domains = domains,

		length = totalLength
	}, CatRom)
end

-- Binary search for the spline in the chain that the t corresponds to.
-- Ex. For an t of 50%, we must find the spline in the chain that contains
-- the 50% mark.
function CatRom:GetSplineFromT(t: number)
	local splines = self.splines
	local domains = self.domains
	local numSplines = #splines

	-- There is only one option if there is one spline
	if numSplines == 1 then
		return splines[1], t
	end

	-- Special cases for when t is on the border or outside of [0, 1]
	if t < 0 then
		return splines[1], t / domains[1], 1
	elseif t == 0 then
		return splines[1], 0, 1
	elseif t == 1 then
		return splines[numSplines], 1, numSplines
	elseif t > 1 then
		return splines[numSplines], (t - domains[numSplines]) / (1 - domains[numSplines]), numSplines
	end

	-- Binary search for the spline containing t
	local left = 1
	local right = numSplines + 1

	while left <= right do
		local mid = math.floor((left + right) / 2)
		local intervalStart = domains[mid]

		if t >= intervalStart then
			local intervalEnd = mid == numSplines and 1 or domains[mid + 1]

			if t <= intervalEnd then
				local spline = splines[mid]
				local splineT = (t - intervalStart) / (intervalEnd - intervalStart)
				return spline, splineT, mid
			else
				left = mid + 1
			end
		else
			right = mid - 1
		end
	end

	-- This should theoretically never be possible, but if it occurs, then we
	-- should not fail silently
	error("Failed to get spline from t")
end

function CatRom:PrecomputeArcLengthParams(numIntervals: number?)
	numIntervals = numIntervals and math.max(1, math.round(numIntervals)) or DEFAULT_PRECOMPUTE_INTERVALS
	for _, spline in ipairs(self.splines) do
		spline:_PrecomputeArcLengthParams(numIntervals)
	end
end

function CatRom:SolveLength(a: number?, b: number?)
	-- Algorithm outline:
	-- Binary search for the two splines that contain the a and b positions
	-- Find where a is in the first spline
	-- Find where b is in the second spline
	-- Get the length from a to the end of the first spline
	-- Get the length from the start of the second spline to b
	-- Sum the full lengths of the splines between the a spline and b spline

	a = a or 0
	b = b or 1

	if a == 0 and b == 1 then
		return self.length
	end

	local splineA, splineAT, splineAIndex = self:GetSplineFromT(a)
	local splineB, splineBT, splineBIndex = self:GetSplineFromT(b)

	local lengthA = splineA:SolveLength(splineAT, 1)
	local lengthB = splineB:SolveLength(0, splineBT)

	local intermediateLengths = 0
	for i = splineAIndex + 1, splineBIndex - 1 do
		intermediateLengths += self.splines[i].length
	end

	return lengthA + intermediateLengths + lengthB
end

function CatRom:SolveUniformLength(a: number?, b: number?)
	a = a or 0
	b = b or 1

	if a == 0 and b == 1 then
		return self.length
	end

	local splineA, splineAT, splineAIndex = self:GetSplineFromT(a)
	local splineB, splineBT, splineBIndex = self:GetSplineFromT(b)

	local lengthA = splineA:SolveUniformLength(splineAT, 1)
	local lengthB = splineB:SolveUniformLength(0, splineBT)

	local intermediateLengths = 0
	for i = splineAIndex + 1, splineBIndex - 1 do
		intermediateLengths += self.splines[i].length
	end

	return lengthA + intermediateLengths + lengthB
end

function CatRom:ComputeMinRotationFrames(pointCount: number)
	assert(pointCount >= 2, "Must generate at least two points!")
	return self:_computeMinRotationFrames(
		pointCount,
		CatRom.SolvePosition,
		CatRom.SolveNormal,
		CatRom.SolveTangent,
		CatRom.SolveBinormal
	)
end

function CatRom:ComputeUniformMinRotationFrames(pointCount: number)
	assert(pointCount >= 2, "Must generate at least two points!")
	return self:_computeMinRotationFrames(
		pointCount,
		CatRom.SolveUniformPosition,
		CatRom.SolveUniformNormal,
		CatRom.SolveUniformTangent,
		CatRom.SolveUniformBinormal
	)
end

function CatRom:_computeMinRotationFrames(pointCount: number, _pos,  _norm, _tan, _binorm)
	--! Curently this is broken for straight lines!
	-- When two points on the curve are in a straight line, x(i+1) - x(i) == t(i).
	-- This means that t(i):Dot(x(i+1)-x(i)) will fail, returning nan.
	-- This breaks the calculations. Not sure how to fix it?

	-- Note that asserts are placed in the calling functions for stack trace purposes.

	-- Utility functions.
	-- Redefining these every function call is wasteful!!
	-- They are used for brevity, and follow the pseudocode in this paper:
	-- https://www.microsoft.com/en-us/research/wp-content/uploads/2016/12/Computation-of-rotation-minimizing-frames.pdf
	-- Note that functions may be inlined in the production / online Roblox player.
	-- i.e. 'toCFrame' is guaranteed to be inlined.
	local function x(fac): Vector3
		return _pos(self, fac)
	end
	local function r(fac): Vector3
		return _norm(self, fac)
	end
	local function t(fac): Vector3
		return _tan(self, fac)
	end
	local function s(fac): Vector3
		return _binorm(self, fac)
	end
	local function toCFrame(pV, uV, lV, rV)
		-- Note that this will flip the lookVector.
		return CFrame.new(pV.X, pV.Y, pV.Z, rV.X, uV.X, -lV.X, rV.Y, uV.Y, -lV.Y, rV.Z, uV.Z, -lV.Z):Orthonormalize()
	end

	local pointArray = table.create(pointCount)

	local ri = r(0)
	local si = s(0)

	for i = 0, pointCount - 2 do
		-- Everything to compute point i.
		local _i = i / (pointCount - 1)
		local xi = x(_i)
		local ti = t(_i)
		pointArray[i + 1] = toCFrame(xi, ri, ti, si)

		-- Everything to compute the next point, i+1.
		-- As we want n points, we should only compute n-1 next points.
		-- Therefore we skip this on the last iteration.
		local _i1 = (i+1) / (pointCount-1)
		local xi1 = x(_i1)
		local ti1 = t(_i1)

		-- From the paper.
		local v1 = xi1 - xi

		local c1 = v1:Dot(v1)
		local rLi = ri - (2 / c1) * (v1:Dot(ri)) * v1
		local tLi = ti - (2 / c1) * (v1:Dot(ti)) * v1

		local v2 = ti1 - tLi
		local c2 = v2:Dot(v2)

		local rNext = rLi - (2 / c2) * (v2:Dot(rLi)) * v2
		local sNext = ti1:Cross(rNext)

		ri = rNext
		si = sNext
	end

	pointArray[pointCount] = toCFrame(x(1), ri, t(1), si)

	return pointArray
end

---- START GENERATED METHODS
function CatRom:SolvePosition(t: number)
	local spline, splineT = self:GetSplineFromT(t)
	return spline:SolvePosition(splineT)
end
function CatRom:SolveVelocity(t: number)
	local spline, splineT = self:GetSplineFromT(t)
	return spline:SolveVelocity(splineT)
end
function CatRom:SolveAcceleration(t: number)
	local spline, splineT = self:GetSplineFromT(t)
	return spline:SolveAcceleration(splineT)
end
function CatRom:SolveTangent(t: number)
	local spline, splineT = self:GetSplineFromT(t)
	return spline:SolveTangent(splineT)
end
function CatRom:SolveNormal(t: number)
	local spline, splineT = self:GetSplineFromT(t)
	return spline:SolveNormal(splineT)
end
function CatRom:SolveBinormal(t: number)
	local spline, splineT = self:GetSplineFromT(t)
	return spline:SolveBinormal(splineT)
end
function CatRom:SolveCurvature(t: number)
	local spline, splineT = self:GetSplineFromT(t)
	return spline:SolveCurvature(splineT)
end
function CatRom:SolveCFrame(t: number)
	local spline, splineT = self:GetSplineFromT(t)
	return spline:SolveCFrame(splineT)
end
function CatRom:SolveRotCFrame(t: number)
	local spline, splineT = self:GetSplineFromT(t)
	return spline:SolveRotCFrame(splineT)
end
function CatRom:SolveUniformPosition(t: number)
	local spline, splineT = self:GetSplineFromT(t)
	return spline:SolveUniformPosition(splineT)
end
function CatRom:SolveUniformVelocity(t: number)
	local spline, splineT = self:GetSplineFromT(t)
	return spline:SolveUniformVelocity(splineT)
end
function CatRom:SolveUniformAcceleration(t: number)
	local spline, splineT = self:GetSplineFromT(t)
	return spline:SolveUniformAcceleration(splineT)
end
function CatRom:SolveUniformTangent(t: number)
	local spline, splineT = self:GetSplineFromT(t)
	return spline:SolveUniformTangent(splineT)
end
function CatRom:SolveUniformNormal(t: number)
	local spline, splineT = self:GetSplineFromT(t)
	return spline:SolveUniformNormal(splineT)
end
function CatRom:SolveUniformBinormal(t: number)
	local spline, splineT = self:GetSplineFromT(t)
	return spline:SolveUniformBinormal(splineT)
end
function CatRom:SolveUniformCurvature(t: number)
	local spline, splineT = self:GetSplineFromT(t)
	return spline:SolveUniformCurvature(splineT)
end
function CatRom:SolveUniformCFrame(t: number)
	local spline, splineT = self:GetSplineFromT(t)
	return spline:SolveUniformCFrame(splineT)
end
function CatRom:SolveUniformRotCFrame(t: number)
	local spline, splineT = self:GetSplineFromT(t)
	return spline:SolveUniformRotCFrame(splineT)
end
---- END GENERATED METHODS

return CatRom
