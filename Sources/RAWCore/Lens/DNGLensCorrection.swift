import Foundation

/// DNG に記録されたレンズ補正値(DNG 標準のオプコード)を読み、LensCorrectionData に換算する。機種には依存しない。
///
/// - OpcodeList2(51009)の GainMap: デモザイク前の画像に掛けるゲインの 2 次元の表(周辺減光)
/// - OpcodeList3(51022)の WarpRectilinear: デモザイク後の画像の放射状の変形。プレーンが 3 つなら色ごと(歪曲と倍率色収差)
///
/// どちらもアクティブエリア(ActiveArea)の座標で書かれている。CIRAWFilter の出力は DefaultCrop で切り抜いた後なので、
/// 中心と半径の取り方を出力の画像に合わせて換算する。今の補正(LensCorrector)は画像の中心について放射状なので、
/// GainMap は半径ごとに角度方向の平均を取って近似し、WarpRectilinear の接線方向の項(kt0・kt1)は使わない。
/// 詳しくは docs/dng-lens-correction.md
public enum DNGLensCorrection {
    /// WarpRectilinear のパラメータ。倍率 f(r) = kr0 + kr1·r² + kr2·r⁴ + kr3·r⁶ で、出力の位置から入力の位置を求める向き
    public struct WarpRectilinear: Equatable, Sendable {
        /// プレーンごとの [kr0, kr1, kr2, kr3, kt0, kt1]
        public var planes: [[Double]]
        /// 光学中心(画像の幅・高さに対する比率。0.5 が中心)
        public var centerX: Double
        public var centerY: Double

        /// プレーン plane の、正規化した半径 r(中心から一番遠い隅までを 1)での倍率
        public func factor(plane: Int, at r: Double) -> Double {
            let k = planes[min(plane, planes.count - 1)]
            let r2 = r * r
            return k[0] + r2 * (k[1] + r2 * (k[2] + r2 * k[3]))
        }
    }

    /// GainMap のパラメータ
    public struct GainMap: Equatable, Sendable {
        /// 掛ける範囲(アクティブエリアの座標、ピクセル)
        public var top: Int, left: Int, bottom: Int, right: Int
        public var plane: Int, planes: Int, rowPitch: Int, colPitch: Int
        public var pointsV: Int, pointsH: Int
        /// 表の点の間隔と原点(範囲の高さ・幅に対する比率)
        public var spacingV: Double, spacingH: Double, originV: Double, originH: Double
        public var mapPlanes: Int
        /// 行(縦)→ 列(横)→ プレーンの順のゲイン
        public var gains: [Float]

        /// アクティブエリアの座標 (x, y) でのゲイン(表の点の間は双線形補間、外は端の値)。プレーンは最初のものを使う
        public func gain(x: Double, y: Double) -> Double {
            let v = (y - Double(top)) / Double(max(bottom - top, 1))
            let h = (x - Double(left)) / Double(max(right - left, 1))
            func index(_ t: Double, _ origin: Double, _ spacing: Double, _ n: Int) -> (Int, Int, Double) {
                guard n > 1, spacing > 0 else { return (0, 0, 0) }
                let f = min(max((t - origin) / spacing, 0), Double(n - 1))
                let i = min(Int(f), n - 2)
                return (i, i + 1, f - Double(i))
            }
            let (r0, r1, fr) = index(v, originV, spacingV, pointsV)
            let (c0, c1, fc) = index(h, originH, spacingH, pointsH)
            func g(_ r: Int, _ c: Int) -> Double { Double(gains[(r * pointsH + c) * mapPlanes]) }
            let top = g(r0, c0) * (1 - fc) + g(r0, c1) * fc
            let bottom = g(r1, c0) * (1 - fc) + g(r1, c1) * fc
            return top * (1 - fr) + bottom * fr
        }
    }

    /// 画像の大きさの関係(すべてアクティブエリアの座標、ピクセル)
    public struct Geometry: Equatable, Sendable {
        /// アクティブエリアの幅・高さ
        public var activeWidth: Double, activeHeight: Double
        /// DefaultCrop(出力の画像)の位置と大きさ
        public var cropX: Double, cropY: Double, cropWidth: Double, cropHeight: Double

        var cropCenter: SIMD2<Double> { SIMD2(cropX + cropWidth / 2, cropY + cropHeight / 2) }
        /// 出力の画像の半対角(LensCorrectionData の半径 1)
        var cropHalfDiagonal: Double { (cropWidth * cropWidth + cropHeight * cropHeight).squareRoot() / 2 }
    }

    /// 読み出したままの値(換算前)。確認・テスト用
    public struct Tags: Equatable, Sendable {
        public var warp: WarpRectilinear?
        public var gainMap: GainMap?
        public var geometry: Geometry
        /// 読まなかったオプコードの番号(OpcodeList2・3 にあったもの)
        public var otherOpcodes: [UInt32]
    }

    public static let warpRectilinearID: UInt32 = 1
    public static let gainMapID: UInt32 = 9

    // MARK: - 読む

    /// RAW の画像の IFD(SubIFD のうち NewSubFileType が 0 のもの)から補正値と大きさを読む
    public static func readTags(from url: URL) -> Tags? {
        guard let tiff = TIFFReader(url: url), let ifd0 = tiff.firstIFDOffset,
              let entries0 = tiff.entries(atIFD: ifd0) else { return nil }
        var candidates = [entries0]
        if let sub = entries0.first(where: { $0.tag == 0x014a }), let offsets = tiff.integers(sub) {
            candidates += offsets.compactMap { tiff.entries(atIFD: $0) }
        }
        for entries in candidates {
            func entry(_ tag: UInt16) -> TIFFReader.Entry? { entries.first { $0.tag == tag } }
            func numbers(_ tag: UInt16) -> [Double]? { entry(tag).flatMap { tiff.numbers($0) } }
            // 縮小画像(NewSubFileType = 1)は飛ばす
            guard numbers(254)?.first ?? 0 == 0,
                  let width = numbers(256)?.first, let height = numbers(257)?.first else { continue }
            let list2 = entry(51009).flatMap { tiff.bytes($0) }.flatMap(parseOpcodes)
            let list3 = entry(51022).flatMap { tiff.bytes($0) }.flatMap(parseOpcodes)
            guard list2 != nil || list3 != nil else { continue }

            // ActiveArea は (上, 左, 下, 右)。無ければ画像全体
            let area = numbers(50829).flatMap { $0.count == 4 ? $0 : nil } ?? [0, 0, height, width]
            let activeW = area[3] - area[1], activeH = area[2] - area[0]
            let origin = numbers(50719).flatMap { $0.count == 2 ? $0 : nil } ?? [0, 0]
            let size = numbers(50720).flatMap { $0.count == 2 ? $0 : nil } ?? [activeW, activeH]
            let geometry = Geometry(activeWidth: activeW, activeHeight: activeH,
                                    cropX: origin[0], cropY: origin[1], cropWidth: size[0], cropHeight: size[1])

            var tags = Tags(warp: nil, gainMap: nil, geometry: geometry, otherOpcodes: [])
            for op in (list2 ?? []) + (list3 ?? []) {
                switch op.id {
                case warpRectilinearID where tags.warp == nil:
                    if let w = warp(from: op.params) { tags.warp = w } else { tags.otherOpcodes.append(op.id) }
                case gainMapID where tags.gainMap == nil:
                    if let g = gainMap(from: op.params) { tags.gainMap = g } else { tags.otherOpcodes.append(op.id) }
                default:
                    tags.otherOpcodes.append(op.id)
                }
            }
            return tags
        }
        return nil
    }

    /// 補正データ。補正値が無い・読めないときは nil
    public static func read(from url: URL) -> LensCorrectionData? {
        readTags(from: url).flatMap(data(from:))
    }

    // MARK: - 換算

    /// 換算に使う点の数(半径 0〜1 を等間隔に)
    static let knotCount = 33

    /// LensCorrectionData に換算する。WarpRectilinear も GainMap も無ければ nil
    public static func data(from tags: Tags) -> LensCorrectionData? {
        guard tags.warp != nil || tags.gainMap != nil else { return nil }
        let g = tags.geometry
        guard g.cropWidth > 0, g.cropHeight > 0, g.activeWidth > 0, g.activeHeight > 0 else { return nil }
        let knots = (0..<knotCount).map { Double($0) / Double(knotCount - 1) }
        let radius = g.cropHalfDiagonal
        let center = g.cropCenter

        // 歪曲・倍率色収差: 出力の画像の半径 r を、WarpRectilinear の半径(光学中心から一番遠い隅までを 1)に直す
        var distortion = [Double](repeating: 1, count: knots.count)
        var red = distortion, blue = distortion
        if let w = tags.warp {
            let c = SIMD2(w.centerX * g.activeWidth, w.centerY * g.activeHeight)
            let corners = [SIMD2(0, 0), SIMD2(g.activeWidth, 0), SIMD2(0, g.activeHeight), SIMD2(g.activeWidth, g.activeHeight)]
            let maxDistance = corners.map { (($0 - c) * ($0 - c)).sum().squareRoot() }.max()!
            let scale = radius / maxDistance
            let greenPlane = w.planes.count >= 3 ? 1 : 0
            for (i, r) in knots.enumerated() {
                let fG = w.factor(plane: greenPlane, at: r * scale)
                distortion[i] = fG
                if w.planes.count >= 3, fG != 0 {
                    red[i] = w.factor(plane: 0, at: r * scale) / fG
                    blue[i] = w.factor(plane: 2, at: r * scale) / fG
                }
            }
        }

        // 周辺減光: 半径 r の円の上(アクティブエリアの中)でゲインを平均し、その逆数を係数にする(明るさを係数で割るため)
        var vignetting = [Double](repeating: 1, count: knots.count)
        if let map = tags.gainMap {
            let steps = 720
            for (i, r) in knots.enumerated() {
                var sum = 0.0, n = 0
                for s in 0..<steps {
                    let a = Double(s) / Double(steps) * 2 * .pi
                    let p = center + SIMD2(cos(a), sin(a)) * (r * radius)
                    guard p.x >= 0, p.y >= 0, p.x <= g.activeWidth, p.y <= g.activeHeight else { continue }
                    sum += map.gain(x: p.x, y: p.y)
                    n += 1
                }
                // 隅より外で円が画像に掛からないときは、一つ内側の値を使う
                vignetting[i] = n > 0 && sum > 0 ? Double(n) / sum : (i > 0 ? vignetting[i - 1] : 1)
            }
        }

        return LensCorrectionData(knots: knots, distortion: distortion, chromaticRed: red, chromaticBlue: blue,
                                  vignetting: vignetting, cameraSettings: .all)
    }

    // MARK: - オプコードの解析(常にビッグエンディアン)

    struct Opcode { var id: UInt32; var flags: UInt32; var params: Data }

    /// オプコードの並び。形が崩れていれば nil
    static func parseOpcodes(_ data: Data) -> [Opcode]? {
        var r = BigEndianReader(data)
        guard let count = r.u32(), count < 1000 else { return nil }
        var ops: [Opcode] = []
        for _ in 0..<count {
            guard let id = r.u32(), r.u32() != nil, let flags = r.u32(), let size = r.u32(),
                  let params = r.bytes(Int(size)) else { return nil }
            ops.append(Opcode(id: id, flags: flags, params: params))
        }
        return ops
    }

    static func warp(from params: Data) -> WarpRectilinear? {
        var r = BigEndianReader(params)
        guard let n = r.u32(), (1...4).contains(n) else { return nil }
        var planes: [[Double]] = []
        for _ in 0..<n {
            var k: [Double] = []
            for _ in 0..<6 { guard let v = r.f64(), v.isFinite else { return nil }; k.append(v) }
            planes.append(k)
        }
        guard let cx = r.f64(), let cy = r.f64(), (0...1).contains(cx), (0...1).contains(cy) else { return nil }
        return WarpRectilinear(planes: planes, centerX: cx, centerY: cy)
    }

    static func gainMap(from params: Data) -> GainMap? {
        var r = BigEndianReader(params)
        var ints: [Int] = []
        for _ in 0..<10 { guard let v = r.u32() else { return nil }; ints.append(Int(v)) }
        guard let spacingV = r.f64(), let spacingH = r.f64(), let originV = r.f64(), let originH = r.f64(),
              let mapPlanes = r.u32().map(Int.init) else { return nil }
        let (pointsV, pointsH) = (ints[8], ints[9])
        guard (1...4096).contains(pointsV), (1...4096).contains(pointsH), (1...4).contains(mapPlanes),
              ints[2] > ints[0], ints[3] > ints[1] else { return nil }
        var gains: [Float] = []
        gains.reserveCapacity(pointsV * pointsH * mapPlanes)
        for _ in 0..<(pointsV * pointsH * mapPlanes) {
            guard let g = r.f32(), g.isFinite, g > 0 else { return nil }
            gains.append(g)
        }
        return GainMap(top: ints[0], left: ints[1], bottom: ints[2], right: ints[3],
                       plane: ints[4], planes: ints[5], rowPitch: ints[6], colPitch: ints[7],
                       pointsV: pointsV, pointsH: pointsH,
                       spacingV: spacingV, spacingH: spacingH, originV: originV, originH: originH,
                       mapPlanes: mapPlanes, gains: gains)
    }
}

/// ビッグエンディアンのバイト列を前から読む。足りなければ nil
struct BigEndianReader {
    private let data: Data
    private var offset = 0

    init(_ data: Data) { self.data = data }

    mutating func bytes(_ n: Int) -> Data? {
        guard n >= 0, offset + n <= data.count else { return nil }
        let start = data.startIndex + offset
        offset += n
        return data.subdata(in: start..<(start + n))
    }

    mutating func u32() -> UInt32? {
        bytes(4).map { $0.reduce(0) { $0 << 8 | UInt32($1) } }
    }

    mutating func u64() -> UInt64? {
        bytes(8).map { $0.reduce(0) { $0 << 8 | UInt64($1) } }
    }

    mutating func f64() -> Double? { u64().map(Double.init(bitPattern:)) }
    mutating func f32() -> Float? { u32().map(Float.init(bitPattern:)) }
}
