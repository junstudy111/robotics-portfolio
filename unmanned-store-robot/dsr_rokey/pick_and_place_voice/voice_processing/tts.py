import os
import time
import pygame
from pathlib import Path
import openai
from dotenv import load_dotenv
from ament_index_python.packages import get_package_share_directory

# -------------------------------------------------
# 🔑 .env에서 OPENAI KEY 로드
# -------------------------------------------------
def load_api_key():
    try:
        pkg_path = get_package_share_directory("pick_and_place_voice")
        env_path = os.path.join(pkg_path, "resource/.env")
        load_dotenv(env_path)
        key = os.getenv("OPENAI_API_KEY")
        return key
    except Exception as e:
        print(f"❌ .env 로드 실패: {e}")
        return None

# -------------------------------------------------
# 전역 변수 (BGM 제어용)
# -------------------------------------------------
BGM_SOUND = None
BGM_CHANNEL = None
CURRENT_BGM_VOLUME = 0.5  # 기본 볼륨 기억용 변수

# -------------------------------------------------
# 🎵 로고송(mp3) 재생 함수
# -------------------------------------------------
def play_logo_music(volume=0.5):
    """
    로고송을 재생하고, 현재 설정된 볼륨을 전역 변수에 저장합니다.
    """
    global BGM_SOUND, BGM_CHANNEL, CURRENT_BGM_VOLUME

    # 현재 볼륨 기억 (나중에 TTS 끝나고 복구할 때 씀)
    CURRENT_BGM_VOLUME = volume

    try:
        pygame.mixer.init()
    except Exception:
        pass

    try:
        pkg_path = get_package_share_directory("pick_and_place_voice")
        mp3_path = os.path.join(pkg_path, "resource/output.mp3")

        print(f"🎶 [MUSIC] 로고송 재생: {mp3_path} (볼륨: {volume})")

        BGM_SOUND = pygame.mixer.Sound(mp3_path)
        BGM_SOUND.set_volume(volume)

        BGM_CHANNEL = pygame.mixer.find_channel()
        if BGM_CHANNEL is None:
            BGM_CHANNEL = pygame.mixer.Channel(5) # 5번 채널 사용
        
        # 채널 볼륨도 확실하게 설정
        BGM_CHANNEL.set_volume(volume)

        # 무한 반복 재생
        BGM_CHANNEL.play(BGM_SOUND, loops=-1)

    except Exception as e:
        print(f"❌ 로고송 재생 실패: {e}")


# -------------------------------------------------
# 🎤 메인 TTS 함수 (덕킹 기능 추가됨)
# -------------------------------------------------
def play_tts(text="어서오세요 슈슈박스입니다!"):
    """
    TTS 재생 시 BGM 소리를 줄이고(Ducking), 끝나면 복구합니다.
    """
    global BGM_CHANNEL, CURRENT_BGM_VOLUME

    api_key = os.getenv("OPENAI_API_KEY") or load_api_key()
    if not api_key:
        print("❌ OPENAI_API_KEY 없음!")
        return False

    openai.api_key = api_key

    try:
        pygame.mixer.init()
    except:
        pass

    print(f"[TTS] 생성 요청 → \"{text}\"")

    try:
        # 1. TTS 생성
        response = openai.audio.speech.create(
            model="tts-1",
            voice="alloy",
            input=text
        )
        tmp_path = Path(__file__).parent / "tts_temp_output.mp3"
        response.stream_to_file(tmp_path)

        # ====================================================
        # 🔻 [덕킹 ON] TTS 시작 전 BGM 줄이기
        # ====================================================
        if BGM_CHANNEL and BGM_CHANNEL.get_busy():
            # 원래 볼륨의 20% 수준으로 줄임 (아주 작게)
            ducking_vol = CURRENT_BGM_VOLUME * 0.2
            BGM_CHANNEL.set_volume(ducking_vol)
            # print(f"🔈 BGM 줄임: {CURRENT_BGM_VOLUME} -> {ducking_vol}")

        # 2. TTS 재생
        pygame.mixer.music.load(tmp_path)
        pygame.mixer.music.play()

        # 3. 재생 끝날 때까지 대기
        while pygame.mixer.music.get_busy():
            time.sleep(0.1)

        # ====================================================
        # 🔺 [덕킹 OFF] TTS 끝나면 BGM 원상복구
        # ====================================================
        if BGM_CHANNEL and BGM_CHANNEL.get_busy():
            BGM_CHANNEL.set_volume(CURRENT_BGM_VOLUME)
            # print(f"🔊 BGM 복구: {CURRENT_BGM_VOLUME}")

        # 4. 파일 정리
        try:
            os.remove(tmp_path)
        except:
            pass

        return True

    except Exception as e:
        print(f"❌ TTS 실패: {e}")
        # 에러 나도 BGM은 복구해줘야 함
        if BGM_CHANNEL:
            BGM_CHANNEL.set_volume(CURRENT_BGM_VOLUME)
        return False


# -------------------------------------------------
# 🔥 BGM 정지 함수
# -------------------------------------------------
def stop_logo_music():
    global BGM_CHANNEL
    try:
        if BGM_CHANNEL:
            BGM_CHANNEL.stop()
    except:
        pass
