param([switch]$Online)
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

$profileRepo = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$profileWork = Join-Path $profileRepo 'build/shooter-config-checks'
New-Item -ItemType Directory -Force -Path $profileWork | Out-Null
$profileInit = Join-Path $profileWork 'classpath.init.gradle'
@'
allprojects { p ->
    if (p.name == 'TeamCode') {
        p.afterEvaluate {
            p.tasks.register('writeShooterTestClasspath') {
                dependsOn('compileDebugJavaWithJavac')
                doLast {
                    def c = p.tasks.named('compileDebugJavaWithJavac').get()
                    def output = new File(p.rootDir, 'build/shooter-config-checks/classpath.txt')
                    output.parentFile.mkdirs()
                    output.text = c.destinationDirectory.get().asFile.absolutePath + File.pathSeparator +
                        c.classpath.asPath + File.pathSeparator + p.android.bootClasspath.join(File.pathSeparator)
                }
            }
        }
    }
}
'@ | Set-Content -LiteralPath $profileInit -Encoding utf8

Push-Location -LiteralPath $profileRepo
try {
    $profileGradleArgs = @('-I', $profileInit, ':TeamCode:writeShooterTestClasspath')
    if (-not $Online) { $profileGradleArgs += '--offline' }
    & (Join-Path $profileRepo 'gradlew.bat') @profileGradleArgs
    if ($LASTEXITCODE -ne 0) { throw 'TeamCode compilation/classpath generation failed.' }
    $profileClasspath = (Get-Content -LiteralPath (Join-Path $profileWork 'classpath.txt') -Raw).Trim()
    $profileClasses = Join-Path $profileWork 'classes'
    New-Item -ItemType Directory -Force -Path $profileClasses | Out-Null
    $profileChecks = @('ShooterConfiguration2026Test', 'Logging2026Test',
        'TurretShotSequence2026Test', 'IntakeLedFeedback2026Test')
    $profileSources = @($profileChecks | ForEach-Object { Join-Path $profileRepo "scripts/tests/$_.java" })
    & javac -cp $profileClasspath -d $profileClasses @profileSources
    if ($LASTEXITCODE -ne 0) { throw 'Desktop test compilation failed.' }
    foreach ($profileCheck in $profileChecks) {
        & java -cp "$profileClasses;$profileClasspath" $profileCheck
        if ($LASTEXITCODE -ne 0) { throw "$profileCheck failed." }
    }
    & python -m unittest discover -s scripts/tests -p test_convert_logs_2026.py -v
    if ($LASTEXITCODE -ne 0) { throw 'Log conversion checks failed.' }
} finally {
    Pop-Location
}
