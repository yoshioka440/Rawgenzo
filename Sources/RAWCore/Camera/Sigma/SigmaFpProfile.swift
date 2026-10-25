import Foundation

/// SIGMA fp 系(fp / fp L)。RAW は DNG
/// - fp: 約 2460 万画素
/// - fp L: 約 6100 万画素(9520×6328)。カメラ側で縮小して記録した DNG(6240×4144 など)もある
///
/// レンズ補正は DNG に記録された値(GainMap・WarpRectilinear)で自前で掛ける(DNGLensCorrection)。
/// CIRAWFilter もこの補正に対応しているが、周辺減光・歪曲・倍率色収差を個別に切れないので使わない
public struct SigmaFpProfile: CameraProfile {
    public let id: String
    public let displayName: String
    /// Exif の Model(大文字小文字は区別しない)
    public let model: String
    public let fileExtensions: Set<String> = ["dng"]

    /// fp(id はサイドカーに保存されるので変えない)
    public static let fp = SigmaFpProfile(id: "sigma.fp", displayName: "SIGMA fp", model: "SIGMA fp")
    /// fp L
    public static let fpL = SigmaFpProfile(id: "sigma.fp-l", displayName: "SIGMA fp L", model: "SIGMA fp L")

    init(id: String, displayName: String, model: String) {
        self.id = id
        self.displayName = displayName
        self.model = model
    }

    public func matches(_ metadata: RAWMetadata) -> Bool {
        guard let make = metadata.make?.uppercased(),
              let model = metadata.model?.trimmingCharacters(in: .whitespaces).uppercased() else { return false }
        return make.hasPrefix("SIGMA") && model == self.model.uppercased()
    }

    public func makeSource(url: URL, metadata: RAWMetadata) throws -> RAWSource {
        try CoreImageRAWSource(url: url, metadata: metadata)
    }

    public func lensCorrection(url: URL, metadata: RAWMetadata) -> LensCorrectionData? {
        DNGLensCorrection.read(from: url)
    }
}
